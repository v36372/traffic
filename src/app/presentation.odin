package app

import "core:fmt"
import "core:math"
import "core:path/filepath"
import sim "../sim"

// Last tick's authoritative poses, keyed by trip ID rather than array index:
// ordered removals and new admissions must never blend unrelated vehicles.
Render_Poses :: map[u64][2]Vec3

remember_vehicle_poses :: proc(previous: ^Render_Poses, world: ^sim.World) {
    clear(previous)
    for vehicle in world.vehicles {
        p, f := vehicle_render_pose(world, vehicle)
        previous[vehicle.id] = {p, f}
    }
}

interpolated_vehicle_pose :: proc(world: ^sim.World, vehicle: sim.Vehicle, previous: Render_Poses, alpha: f32) -> (position, forward: Vec3) {
    position, forward = vehicle_render_pose(world, vehicle)
    old, found := previous[vehicle.id]
    if !found || alpha >= 1 { return }
    if alpha <= 0 { return old[0], old[1] }
    // Short tick chords are checked by the turn-containment tests. Coarser
    // ticks/tighter paths would need simulation-path resampling instead.
    position = old[0]+(position-old[0])*alpha
    // Interpolate yaw along the shortest arc, including the -pi/pi seam.
    before := math.atan2(old[1][0], old[1][2])
    after := math.atan2(forward[0], forward[2])
    delta := math.atan2(math.sin(after-before), math.cos(after-before))
    yaw := before+delta*alpha
    forward = {math.sin(yaw), 0, math.cos(yaw)}
    return
}

Scene_Profile :: enum { Fitted, Showcase, Intersection }

same_map_path :: proc(path, expected: string) -> bool {
    actual, actual_error := filepath.abs(path)
    defer delete(actual)
    want, want_error := filepath.abs(expected)
    defer delete(want)
    return actual_error == nil && want_error == nil && actual == want
}

is_showcase_map :: proc(path: string) -> bool {
    return same_map_path(path, SHOWCASE_MAP_PATH)
}

is_default_map :: proc(path: string) -> bool {
    return same_map_path(path, DEFAULT_MAP_PATH)
}

scene_profile :: proc(path: string) -> Scene_Profile {
    if is_showcase_map(path) { return .Showcase }
    if is_default_map(path) { return .Intersection }
    return .Fitted
}

// Authored entry compositions, not whole-map fits. Home keeps the separate
// full-scene overview; physical vehicle/signal sizes are fixed.
scene_entry_view :: proc(world: ^sim.World, buildings: ^Building_Scene, profile: Scene_Profile, aspect: f32 = 1.6) -> View {
    switch profile {
    case .Showcase:
        return View{
            target = {-45, 0, 0}, yaw = -0.12, pitch = 1.30,
            distance = 150/(2*math.tan(f32(math.PI/8))*math.max(aspect, 0.1)),
            extent = 150,
        }
    case .Intersection:
        return View{
            target = {0, 0, 0}, yaw = -0.08, pitch = 1.22,
            distance = 110/(2*math.tan(f32(math.PI/8))*math.max(aspect, 0.1)),
            extent = 110,
        }
    case .Fitted:
        return fit_scene_view(world, buildings, aspect)
    }
    return fit_scene_view(world, buildings, aspect)
}

// Raylib's default 0.05m near plane gives <1 24-bit depth unit between our
// 15mm pavement layers at the 113m entry view. A 2%-of-orbit near plane gives
// ~43 units without inflating geometry; scale far to include the ground plane.
// Very shallow, horizon-length views still have finite depth precision.
camera_clip_planes :: proc(view: View) -> (near, far: f64) {
    near = math.max(0.1, f64(view.distance)*0.02)
    far = math.max(1000, f64(view.distance)+math.max(f64(view.extent)*4, f64(view.distance)*3))
    return
}

warmup_scene :: proc(world: ^sim.World, profile: Scene_Profile, smoke: bool, seconds: f64 = -1) -> int {
    ticks := 0
    switch profile {
    case .Showcase: ticks = 1200
    case .Intersection: ticks = 600
    case .Fitted: if smoke { ticks = 400 }
    }
    if seconds >= 0 { ticks = int(math.round(seconds/TICK)) }
    // Advance the same seeded simulation in normal and capture launches. Never
    // inject display-only traffic; explicit fixtures/reset retain their zero start.
    for _ in 0..<ticks { sim.step(world, f32(TICK)) }
    if ticks > 0 {
        fmt.printf("Warmup: %d fixed ticks, time=%.2f, active=%d, spawned=%d, completed=%d\n", ticks, world.time, len(world.vehicles), world.spawned, world.completed)
    }
    return ticks
}
