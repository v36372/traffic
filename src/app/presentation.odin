package app

import "core:fmt"
import "core:math"
import "core:path/filepath"
import sim "../sim"

is_showcase_map :: proc(path: string) -> bool {
    actual, actual_error := filepath.abs(path)
    defer delete(actual)
    expected, expected_error := filepath.abs(DEFAULT_MAP_PATH)
    defer delete(expected)
    return actual_error == nil && expected_error == nil && actual == expected
}

// Authored showcase entry composition, not a whole-map fit. The west signalized
// crossing and market-facing blocks occupy ~150m horizontally. Home deliberately
// keeps the separate full-scene overview; physical vehicle/signal sizes are fixed.
scene_entry_view :: proc(world: ^sim.World, buildings: ^Building_Scene, showcase: bool, aspect: f32 = 1.6) -> View {
    if !showcase { return fit_scene_view(world, buildings, aspect) }
    return View{
        target = {-45, 0, 0}, yaw = -0.12, pitch = 1.30,
        distance = 150/(2*math.tan(f32(math.PI/8))*math.max(aspect, 0.1)),
        extent = 150,
    }
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

warmup_scene :: proc(world: ^sim.World, showcase, smoke: bool, seconds: f64 = -1) -> int {
    ticks := 0
    if showcase { ticks = 1200 } else if smoke { ticks = 400 }
    if seconds >= 0 { ticks = int(math.round(seconds/TICK)) }
    // Advance the same seeded simulation in normal and capture launches. Never
    // inject display-only traffic; explicit fixtures/reset retain their zero start.
    for _ in 0..<ticks { sim.step(world, f32(TICK)) }
    if ticks > 0 {
        fmt.printf("Warmup: %d fixed ticks, time=%.2f, active=%d, spawned=%d, completed=%d\n", ticks, world.time, len(world.vehicles), world.spawned, world.completed)
    }
    return ticks
}
