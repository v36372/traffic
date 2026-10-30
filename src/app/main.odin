package app

import "core:fmt"
import "core:math"
import "core:os"
import "core:strings"

import sim "../sim"
import mapdata "../mapdata"
import rl "vendor:raylib"
import gl "vendor:raylib/rlgl"

Vec3 :: rl.Vector3
TICK :: f64(0.05)
DEFAULT_MAP_PATH :: "build/maps/showcase.csv"
INK :: rl.Color{16, 27, 35, 240}
PAPER :: rl.Color{227, 235, 232, 255}
MINT :: rl.Color{125, 231, 190, 255}
ASPHALT :: rl.Color{62, 71, 77, 255}
GLASS :: rl.Color{41, 78, 93, 255}
RUBBER :: rl.Color{24, 29, 34, 255}

View :: struct {
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
    extent: f32,
}

main :: proc() {
    code := run_frontend()
    if code != 0 { os.exit(code) }
}

run_frontend :: proc() -> int {
    // Parse before initializing raylib: --help works without a display.
    for arg in os.args[1:] {
        if arg == "--help" || arg == "-h" {
            fmt.println("HCMC Traffic | Phase A technical prototype\nUsage: traffic-game [--map path] [--smoke-test] [--screenshot path.png]\n                   [--demand-scale multiplier] [--warmup seconds] [--capture-sequence DIR]\n\nDefault: authored synthetic neighborhood (build/maps/showcase.csv; required).\nShowcase starts at 60s with a focused west-crossing camera; Home shows the full map.\n--map loads geometry and optional stem-objects.csv signals/demand.\nWithout spawn definitions, demand is random synthetic (not calibrated).\n--demand-scale >0..20 divides configured source intervals; result must stay 0.05..86400s.\nThis is synthetic demand, not measured busy/off-peak counts; requires sources unless scale=1.\n--warmup 0..3600s, rounded to 50ms ticks; default showcase 60s, smoke 20s, otherwise 0s.\n--smoke-test renders 90 fixed-clock frames, then exits; no live controls.\n--screenshot captures frame 90 (creates no parent directories).\n--capture-sequence DIR implies smoke: 600 frames / 10s at 60fps, PNG every 6 frames (10Hz).\nDIR must already exist and be writable; captures frame-0006.png through frame-0600.png.\nUse the same map and warmup for synthetic demand comparisons; camera is unchanged.\n\nWASD/arrows or right drag: pan | Middle drag: orbit | Wheel: zoom\nN: north | Home: full-map overview | Space: pause | +/-: speed (0.25x..8x)\nR: restart current configuration | M: load map (--map path or build/maps/showcase.csv)\nB: synthetic demo (clears objects) | Esc: quit\n\nMap attribution: (c) OpenStreetMap contributors, ODbL\nhttps://www.openstreetmap.org/copyright")
            return 0
        }
    }
    options, options_ok := parse_options(os.args[1:])
    if !options_ok { return 2 }
    smoke, map_path, screenshot := options.smoke, options.map_path, options.screenshot
    if options.sequence_dir != "" && !sequence_directory_ready(options.sequence_dir) {
        fmt.eprintf("Capture directory must exist and be writable: %s\n", options.sequence_dir)
        return 2
    }

    showcase := is_showcase_map(map_path)
    world: sim.World
    defer sim.destroy(&world)
    map_mode := load_map(&world, map_path, options.demand_scale)
    if !map_mode { return 1 }
    map_error := false
    buildings: Building_Scene
    if map_mode { buildings = load_building_scene(map_path) }
    defer destroy_building_scene(&buildings)
    offsets := road_offsets(&world)
    defer delete(offsets)
    roads := build_road_scene(&world, offsets)
    defer destroy_road_scene(&roads)
    approaches := signal_approaches(&world, offsets)
    defer delete(approaches)
    view := scene_entry_view(&world, &buildings, showcase)
    paused := false
    speed: f64 = 1
    accumulator: f64
    warmup_scene(&world, showcase, smoke, options.warmup)

    rl.SetConfigFlags({.WINDOW_RESIZABLE, .MSAA_4X_HINT})
    rl.InitWindow(1280, 800, "HCMC Traffic | Phase A")
    if !rl.IsWindowReady() { fmt.eprintln("Window initialization failed; capture unavailable"); return 1 }
    defer rl.CloseWindow()
    rl.SetWindowMinSize(960, 640)
    rl.SetTargetFPS(CAPTURE_FPS)
    frames := 0
    screenshot_written := false
    for !rl.WindowShouldClose() {
        // Drop long wall-clock stalls instead of unbounded catch-up (40 ticks/frame
        // at 8x maximum); simulation itself always advances in fixed 50ms steps.
        dt := math.min(rl.GetFrameTime(), 0.25)
        // Smoke runs ignore live desktop controls so their seeded state is reproducible.
        if !smoke {
            if rl.IsKeyPressed(.SPACE) { paused = !paused; accumulator = 0 }
            if rl.IsKeyPressed(.EQUAL) || rl.IsKeyPressed(.KP_ADD) { speed = math.min(speed*2, 8) }
            if rl.IsKeyPressed(.MINUS) || rl.IsKeyPressed(.KP_SUBTRACT) { speed = math.max(speed*0.5, 0.25) }
            if rl.IsKeyPressed(.R) {
                if reset_world(&world) { accumulator = 0 }
            }
            scene_changed := false
            if rl.IsKeyPressed(.M) {
                if load_map(&world, map_path, options.demand_scale) { map_mode = true; map_error = false; scene_changed = true } else { map_error = true }
            }
            if rl.IsKeyPressed(.B) {
                sim.destroy(&world); sim.init_demo(&world)
                map_mode = false; map_error = false; scene_changed = true
            }
            if scene_changed {
                destroy_building_scene(&buildings)
                if map_mode { buildings = load_building_scene(map_path) }
                delete(offsets); offsets = road_offsets(&world)
                destroy_road_scene(&roads); roads = build_road_scene(&world, offsets)
                delete(approaches); approaches = signal_approaches(&world, offsets)
                view = scene_entry_view(&world, &buildings, showcase && map_mode, f32(rl.GetScreenWidth())/f32(rl.GetScreenHeight()))
                if map_mode { warmup_scene(&world, showcase, false, options.warmup) }
                accumulator = 0
            }
            if rl.IsKeyPressed(.HOME) { view = fit_scene_view(&world, &buildings, f32(rl.GetScreenWidth())/f32(rl.GetScreenHeight())) }
            if rl.IsKeyPressed(.N) { view.yaw = 0 }
            update_view(&view, dt)
        }
        if smoke {
            advance_capture_frame(&world, frames+1)
        } else if !paused {
            accumulator += f64(dt)*speed
            for accumulator >= TICK { sim.step(&world, f32(TICK)); accumulator -= TICK }
        }
        camera := view_camera(view)
        near, far := camera_clip_planes(view)
        gl.SetClipPlanes(near, far)
        rl.BeginDrawing()
        rl.ClearBackground(rl.Color{145, 177, 184, 255})
        rl.BeginMode3D(camera)
        ground := view.target; ground[1] = -0.12
        ground_size := math.max(view.extent*5, view.distance*4)
        rl.DrawPlane(ground, {ground_size, ground_size}, rl.Color{148, 153, 143, 255})
        draw_buildings(&buildings)
        draw_road_scene(&roads)
        draw_signals(&world, approaches[:])
        for vehicle in world.vehicles { draw_vehicle(&world, vehicle) }
        rl.EndMode3D()
        draw_hud(&world, map_mode, map_error, paused, speed, view, buildings.count, showcase, options.demand_scale)
        rl.EndDrawing()
        frames += 1
        if frames == 90 && screenshot != "" {
            screenshot_written = export_screenshot(screenshot)
            if !screenshot_written { return 1 }
        }
        if options.sequence_dir != "" && capture_due(frames) {
            path := fmt.tprintf("%s/frame-%04d.png", options.sequence_dir, frames)
            if !export_screenshot(path) { return 1 }
        }
        // Formatted HUD text and screenshot path allocations live for one frame.
        free_all(context.temp_allocator)
        if smoke && frames >= (SEQUENCE_FRAMES if options.sequence_dir != "" else 90) { break }
    }
    fmt.printf("Frontend closed: frames=%d, sim_time=%.2f, active=%d\n", frames, world.time, len(world.vehicles))
    if options.sequence_dir != "" && frames < SEQUENCE_FRAMES {
        fmt.eprintf("Capture sequence incomplete: %d/%d frames\n", frames, SEQUENCE_FRAMES)
        return 1
    }
    if screenshot != "" && !screenshot_written {
        fmt.eprintf("Screenshot request not fulfilled: %s\n", screenshot)
        return 1
    }
    return 0
}


road_bounds :: proc(world: ^sim.World) -> (low, high: Vec3) {
    if len(world.nodes) == 0 { return {-15, 0, -15}, {15, 0, 15} }
    n := world.nodes[0].position
    low = {n.x, 0, -n.y}; high = low
    for node in world.nodes {
        low[0] = math.min(low[0], node.position.x); low[2] = math.min(low[2], -node.position.y)
        high[0] = math.max(high[0], node.position.x); high[2] = math.max(high[2], -node.position.y)
    }
    margin: f32 = 2.2
    for road in world.roads { margin = math.max(margin, road.width+2.2) }
    low -= {margin, 0, margin}; high += {margin, 0, margin}
    return
}

// Fit actual 3D bounds in camera space, not a padded world-space square.
// Padding reserves the compact top/bottom HUD; aspect handles portrait windows.
fit_bounds_view :: proc(low, high: Vec3, aspect: f32 = 1.6) -> View {
    view := View{target = (low+high)*0.5, yaw = 0.18, pitch = 1.03, extent = math.max(30, math.max(high[0]-low[0], high[2]-low[2]))}
    back := Vec3{math.cos(view.pitch)*math.sin(view.yaw), math.sin(view.pitch), math.cos(view.pitch)*math.cos(view.yaw)}
    right := Vec3{math.cos(view.yaw), 0, -math.sin(view.yaw)}
    up := Vec3{-math.sin(view.pitch)*math.sin(view.yaw), math.cos(view.pitch), -math.sin(view.pitch)*math.cos(view.yaw)}
    tan_y := math.tan(f32(45*math.PI/360))*0.82
    tan_x := math.tan(f32(45*math.PI/360))*math.max(aspect, 0.1)*0.9
    view.distance = 8
    for x in ([2]f32{low[0], high[0]}) {
        for y in ([2]f32{low[1], high[1]}) {
            for z in ([2]f32{low[2], high[2]}) {
                p := Vec3{x, y, z}-view.target
                depth := p[0]*back[0]+p[1]*back[1]+p[2]*back[2]
                horizontal := math.abs(p[0]*right[0]+p[2]*right[2])/tan_x
                vertical := math.abs(p[0]*up[0]+p[1]*up[1]+p[2]*up[2])/tan_y
                view.distance = math.max(view.distance, depth+math.max(horizontal, vertical))
            }
        }
    }
    return view
}

fit_view :: proc(world: ^sim.World, aspect: f32 = 1.6) -> View {
    low, high := road_bounds(world)
    return fit_bounds_view(low, high, aspect)
}

view_camera :: proc(view: View) -> rl.Camera3D {
    return rl.Camera3D{
        position = view.target + Vec3{view.distance*math.cos(view.pitch)*math.sin(view.yaw), view.distance*math.sin(view.pitch), view.distance*math.cos(view.pitch)*math.cos(view.yaw)},
        target = view.target, up = {0, 1, 0}, fovy = 45, projection = .PERSPECTIVE,
    }
}

update_view :: proc(view: ^View, dt: f32) {
    delta := rl.GetMouseDelta()
    if rl.IsMouseButtonDown(.MIDDLE) {
        view.yaw -= delta[0]*0.006
        view.pitch = math.clamp(view.pitch+delta[1]*0.005, 0.25, 1.48)
    }
    view.distance = math.clamp(view.distance*math.pow(0.86, rl.GetMouseWheelMove()), 8, math.max(350, view.extent*4))
    right := Vec3{math.cos(view.yaw), 0, -math.sin(view.yaw)}
    forward := Vec3{-math.sin(view.yaw), 0, -math.cos(view.yaw)}
    pan := view.distance*0.65*dt
    if rl.IsKeyDown(.A) || rl.IsKeyDown(.LEFT) { view.target -= right*pan }
    if rl.IsKeyDown(.D) || rl.IsKeyDown(.RIGHT) { view.target += right*pan }
    if rl.IsKeyDown(.W) || rl.IsKeyDown(.UP) { view.target += forward*pan }
    if rl.IsKeyDown(.S) || rl.IsKeyDown(.DOWN) { view.target -= forward*pan }
    if rl.IsMouseButtonDown(.RIGHT) { view.target += (right*(-delta[0])+forward*delta[1])*(view.distance*0.0012) }
}

// Reciprocal directed edges share a source centerline. Present them on opposite
// halves of a street; this visual offset does not mutate simulation coordinates.
// O(n²) once per load is sufficient for the tiny Phase A map; index endpoint pairs
// before loading district-scale networks.
road_offsets :: proc(world: ^sim.World) -> []f32 {
    offsets := make([]f32, len(world.roads))
    for _, i in world.roads { offsets[i] = sim.road_offset(world, i) }
    return offsets
}

road_frame :: proc(world: ^sim.World, road: sim.Road) -> (start, forward, left: Vec3, length: f32) {
    a, b := world.nodes[road.from].position, world.nodes[road.to].position
    // Importer uses east/north. Raylib +Z is south so a north-up view is not mirrored.
    start = {a.x, 0, -a.y}
    dx, dz := b.x-a.x, a.y-b.y
    length = math.sqrt(dx*dx+dz*dz)
    if length > 0 { forward = {dx/length, 0, dz/length}; left = {dz/length, 0, -dx/length} }
    return
}

strip :: proc(a, b, left: Vec3, width: f32, color: rl.Color) {
    side := left*(width*0.5)
    rl.DrawTriangle3D(a-side, b-side, b+side, color)
    rl.DrawTriangle3D(a-side, b+side, a+side, color)
}


vehicle_color :: proc(kind: sim.Vehicle_Kind, id: u64) -> rl.Color {
    // Reference-supported body variety, not a measured fleet/color distribution.
    bodies := [6]rl.Color{{177, 185, 185, 255}, {225, 228, 220, 255}, {46, 57, 64, 255}, {70, 124, 157, 255}, {158, 65, 57, 255}, {48, 129, 100, 255}}
    switch kind {
    case .Motorcycle: return rl.Color{247, 191, 77, 255}
    case .Car, .Bus, .Truck: return bodies[int(id%u64(len(bodies)))]
    case .Bicycle: return rl.Color{133, 225, 148, 255}
    case .Tricycle: return rl.Color{240, 147, 76, 255}
    }
    return rl.WHITE
}

// Reference-supported variety, not a measured color distribution. Successful
// admissions receive stable seeded IDs; never sample physics RNG while drawing.
rider_colors :: proc(id: u64) -> (shirt, helmet, body: rl.Color) {
    shirts := [6]rl.Color{{41, 153, 175, 255}, {213, 88, 56, 255}, {224, 223, 210, 255}, {48, 62, 72, 255}, {163, 168, 163, 255}, {221, 174, 67, 255}}
    helmets := [6]rl.Color{{225, 231, 223, 255}, {37, 49, 66, 255}, {186, 63, 60, 255}, {75, 143, 160, 255}, {227, 222, 209, 255}, {48, 56, 65, 255}}
    bodies := [3]rl.Color{{53, 64, 74, 255}, {161, 61, 54, 255}, {207, 213, 207, 255}}
    return shirts[int(id%u64(len(shirts)))], helmets[int(id%u64(len(helmets)))], bodies[int(id%u64(len(bodies)))]
}

wheel :: proc(x, z, radius: f32) {
    rl.DrawCylinderEx({x-0.09, radius+0.04, z}, {x+0.09, radius+0.04, z}, radius, radius, 8, RUBBER)
}

vehicle_render_pose :: proc(world: ^sim.World, v: sim.Vehicle) -> (position, forward: Vec3) {
    point, heading := sim.vehicle_pose(world, v)
    // One east/north -> east/up/south conversion; sim already owns lateral and
    // carriageway offsets as well as junction paths. Never offset/blend again.
    return {point.x, 0, -point.y}, {heading.x, 0, -heading.y}
}

draw_vehicle :: proc(world: ^sim.World, v: sim.Vehicle) {
    if v.road < 0 || v.road >= len(world.roads) { return }
    position, forward := vehicle_render_pose(world, v)
    color := vehicle_color(v.kind, v.id)
    width, body_length := sim.kind_width(v.kind), sim.kind_length(v.kind)
    gl.PushMatrix()
    defer gl.PopMatrix()
    gl.Translatef(position[0], 0.06, position[2])
    gl.Rotatef(math.atan2(forward[0], forward[2])*180/math.PI, 0, 1, 0)
    // Models are meter-scale and face local +Z; shape, not only color, identifies class.
    switch v.kind {
    case .Motorcycle, .Bicycle:
        radius: f32 = 0.27
        if v.kind == .Bicycle { radius = 0.33 }
        wheel(0, body_length*0.34, radius); wheel(0, -body_length*0.34, radius)
        shirt, helmet, body := rider_colors(v.id)
        rl.DrawCylinderEx({0, 0.45, -0.55}, {0, 0.65, 0.5}, 0.09, 0.09, 6, body)
        if v.kind == .Motorcycle { rl.DrawCubeV({0, 0.53, 0.1}, {width*0.6, 0.38, 0.8}, body) }
        rl.DrawCubeV({0, 0.78, -0.23}, {0.34, 0.14, 0.6}, RUBBER)
        rl.DrawCubeV({0, 1.02, -0.16}, {0.35, 0.48, 0.35}, shirt)
        // Pexels street references show helmeted riders with bent limbs, not
        // enlarged bikes. Keep the helmet 0.34m wide and the existing footprint.
        rl.DrawSphereEx({0, 1.42, -0.08}, 0.17, 8, 8, helmet)
        rl.DrawCubeV({0, 1.41, 0.075}, {0.23, 0.08, 0.04}, GLASS)
        for side in ([]f32{-1, 1}) {
            shoulder := Vec3{side*0.16, 1.18, -0.10}
            elbow := Vec3{side*0.23, 1.00, 0.14}
            hand := Vec3{side*0.25, 0.95, 0.48}
            rl.DrawCylinderEx(shoulder, elbow, 0.055, 0.055, 6, shirt)
            rl.DrawCylinderEx(elbow, hand, 0.05, 0.045, 6, shirt)
            hip := Vec3{side*0.13, 0.84, -0.24}
            knee := Vec3{side*0.22, 0.61, 0.20}
            foot := Vec3{side*0.22, 0.30, 0.04}
            rl.DrawCylinderEx(hip, knee, 0.075, 0.065, 6, INK)
            rl.DrawCylinderEx(knee, foot, 0.065, 0.055, 6, INK)
        }
        rl.DrawCylinderEx({-width*0.45, 0.95, 0.48}, {width*0.45, 0.95, 0.48}, 0.04, 0.04, 6, PAPER)
    case .Tricycle:
        wheel(0, 0.95, 0.25); wheel(-0.52, -0.8, 0.25); wheel(0.52, -0.8, 0.25)
        rl.DrawCubeV({0, 0.7, -0.5}, {width*0.9, 0.65, 1.4}, color)
        rl.DrawCubeV({0, 1.04, -0.5}, {width*0.7, 0.06, 1.15}, RUBBER)
        rl.DrawCubeV({0, 0.77, 0.65}, {0.4, 0.5, 0.7}, color)
        rl.DrawSphere({0, 1.35, 0.48}, 0.22, PAPER)
    case .Car, .Bus, .Truck:
        for side in ([]f32{-1, 1}) {
            wheel(side*(width*0.46), body_length*0.31, 0.35)
            wheel(side*(width*0.46), -body_length*0.31, 0.35)
        }
        if v.kind == .Car {
            rl.DrawCubeV({0, 0.62, 0}, {width*0.94, 0.65, body_length}, color)
            rl.DrawCubeV({0, 1.14, -0.15}, {width*0.8, 0.65, body_length*0.53}, GLASS)
            rl.DrawCubeV({0, 1.49, -0.2}, {width*0.83, 0.12, body_length*0.4}, color)
        } else if v.kind == .Bus {
            rl.DrawCubeV({0, 1.3, 0}, {width, 2.15, body_length}, color)
            rl.DrawCubeV({0, 1.75, body_length*0.501}, {width*0.84, 0.9, 0.03}, GLASS)
            for side in ([]f32{-1, 1}) {
                for j in 0..<6 { rl.DrawCubeV({side*width*0.502, 1.8, -body_length*0.35+f32(j)*body_length*0.135}, {0.03, 0.8, body_length*0.105}, GLASS) }
            }
            rl.DrawCubeV({0, 2.42, 0}, {width*0.95, 0.12, body_length*0.97}, PAPER)
        } else {
            rl.DrawCubeV({0, 1.25, -0.8}, {width, 2.1, body_length*0.7}, color)
            rl.DrawCubeV({0, 0.95, body_length*0.36}, {width*0.94, 1.5, body_length*0.24}, PAPER)
            rl.DrawCubeV({0, 1.32, body_length*0.485}, {width*0.8, 0.62, 0.03}, GLASS)
        }
        for side in ([]f32{-1, 1}) {
            rl.DrawCubeV({side*width*0.32, 0.62, body_length*0.503}, {0.25, 0.18, 0.04}, rl.Color{255, 241, 185, 255})
            rl.DrawCubeV({side*width*0.32, 0.62, -body_length*0.503}, {0.25, 0.18, 0.04}, rl.Color{224, 78, 65, 255})
        }
    }
}

draw_hud :: proc(world: ^sim.World, map_mode, map_error, paused: bool, speed: f64, view: View, building_count: int, showcase: bool, demand_scale: f32) {
    w, h := rl.GetScreenWidth(), rl.GetScreenHeight()
    rl.DrawRectangle(16, 16, 422, 91, INK)
    rl.DrawRectangle(16, 16, 3, 91, MINT)
    rl.DrawText("TRAFFIC LAB / PROTOTYPE", 30, 26, 18, PAPER)
    label: cstring = "DEBUG JUNCTION / SYNTHETIC DEMAND"
    if map_mode { label = fmt.ctprintf("MAP / %d BLDGS / SYNTHETIC DEMAND %.2gx", building_count, demand_scale) }
    if map_mode && showcase { label = fmt.ctprintf("AUTHORED / %d BLDGS / SYNTHETIC DEMAND %.2gx", building_count, demand_scale) }
    rl.DrawText(label, 30, 51, 14, MINT)
    state := "RUN"; if paused { state = "PAUSED" }
    rl.DrawText(fmt.ctprintf("%s %.2gx  %.1fs  |  %d active  %d complete", state, speed, world.time, len(world.vehicles), world.completed), 30, 76, 16, PAPER)
    rl.DrawRectangle(w-176, 16, 160, 49, INK)
    rl.DrawText(fmt.ctprintf("LIGHTS %d  QUEUED %d", len(world.signals), world.pending), w-166, 25, 13, PAPER)
    rl.DrawText(fmt.ctprintf("VIEW %.0fm / Home: fit", view.distance), w-166, 45, 13, MINT)
    rl.DrawRectangle(0, h-53, w, 53, INK)
    rl.DrawText("WASD / drag: pan   MMB: orbit   Wheel: zoom   Home: fit   N: north   Space: pause   +/-: speed   R: reset   M: map   B: debug", 16, h-44, 14, PAPER)
    rl.DrawText("(c) OpenStreetMap contributors / ODbL / openstreetmap.org/copyright | Synthetic traffic; crossing paint only, no pedestrian simulation", 16, h-22, 13, MINT)
    if map_error {
        rl.DrawRectangle(16, 115, 530, 30, INK)
        rl.DrawText("Map load failed; prior scene retained. See terminal.", 28, 123, 16, rl.Color{255, 151, 116, 255})
    }
    if len(world.motion_issues) > 0 {
        issue := world.motion_issues[0]
        y := i32(153 if map_error else 115)
        rl.DrawRectangle(16, y, 740, 30, INK)
        rl.DrawText(fmt.ctprintf("MOTION ISSUES %d | %d -> %d: %v | held upstream; see terminal", len(world.motion_issues), issue.road, issue.exit, issue.problem), 28, y+8, 14, rl.Color{255, 151, 116, 255})
    }
}
