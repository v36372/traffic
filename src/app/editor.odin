package app

import "core:fmt"
import "core:math"
import "core:strconv"
import "core:strings"
import sim "../sim"
import replay "../replay"
import rl "vendor:raylib"
import gl "vendor:raylib/rlgl"

EDITOR_WIDTH :: 310
EDITOR_BATCH_TICKS :: 40
Editor_Action :: enum { None, Save, Load, Check }
Editor :: struct {
    selected: u64,
    run_to: f64,
    run_to_text: [32]u8,
    run_to_len: int,
    editing_time, seeking: bool,
    result: [2][96]u8,
    result_len: [2]int,
    probe: replay.Steering_Probe,
    probe_available, probe_cached: bool,
    probe_time: f64,
    probe_id: u64,
}

editor_init :: proc() -> Editor {
    e := Editor{run_to = 250, run_to_len = 3}
    copy(e.run_to_text[:], "250")
    return e
}

editor_select_next :: proc(e: ^Editor, world: ^sim.World) {
    if len(world.vehicles) == 0 { e.selected = 0; return }
    index := 0
    for v, i in world.vehicles { if v.id == e.selected { index = (i+1)%len(world.vehicles); e.selected = world.vehicles[index].id; return } }
    e.selected = world.vehicles[0].id
}

editor_select_nearest :: proc(e: ^Editor, world: ^sim.World, camera: rl.Camera3D, mouse: rl.Vector2, sidebar: i32) -> bool {
    if mouse.x >= f32(sidebar) || len(world.vehicles) == 0 { return false }
    best: f32 = 22*22
    found: u64
    for v in world.vehicles {
        p, _ := vehicle_render_pose(world, v)
        screen := rl.GetWorldToScreen(p+Vec3{0, 0.7, 0}, camera)
        dx, dy := screen.x-mouse.x, screen.y-mouse.y
        d := dx*dx+dy*dy
        if d < best { best, found = d, v.id }
    }
    if found == 0 { return false }
    e.selected = found
    return true
}

editor_vehicle :: proc(world: ^sim.World, id: u64) -> (sim.Vehicle, bool) {
    for v in world.vehicles { if v.id == id { return v, true } }
    return {}, false
}

editor_tick :: proc(world: ^sim.World, previous: ^Render_Poses) {
    remember_vehicle_poses(previous, world)
    sim.step(world, f32(TICK))
}

editor_finish_seek :: proc(e: ^Editor, world: ^sim.World) {
    fmt.printf("EDITOR SEEK target=%.2f actual=%.2f\n", e.run_to, world.time)
    e.seeking = false
}

editor_seek_frame :: proc(e: ^Editor, world: ^sim.World, previous: ^Render_Poses) {
    if !e.seeking { return }
    for i in 0..<EDITOR_BATCH_TICKS {
        if world.time >= e.run_to { editor_finish_seek(e, world); return }
        editor_tick(world, previous)
    }
    if world.time >= e.run_to { editor_finish_seek(e, world) }
}

editor_body_bounds :: proc(position, forward: Vec3, kind: sim.Vehicle_Kind, color: rl.Color) {
    gl.PushMatrix()
    defer gl.PopMatrix()
    gl.Translatef(position.x, position.y, position.z)
    gl.Rotatef(math.atan2(forward.x, forward.z)*180/math.PI, 0, 1, 0)
    rl.DrawCubeWiresV({}, {sim.kind_width(kind), 2.5, sim.kind_length(kind)}, color)
}

editor_is_blocker :: proc(world: ^sim.World, selected, other: sim.Vehicle) -> bool {
    if selected.id == other.id { return false }
    for sample in 0..=6 {
        seconds := f32(sample)*0.25
        a := sim.forecast_vehicle(world, selected, max(selected.speed, 1)*seconds)
        b := sim.forecast_vehicle(world, other, other.speed*seconds)
        if sim.bodies_overlap(world, a, b) { return true }
    }
    return false
}

editor_parse_target :: proc(e: ^Editor) -> (f64, bool) {
    value, ok := strconv.parse_f64(string(e.run_to_text[:e.run_to_len]))
    if !ok || math.is_nan(value) || math.is_inf(value) || value < 0 || value > 3600 { return 0, false }
    return math.round(value/TICK)*TICK, true
}

editor_refresh_probe :: proc(e: ^Editor, world: ^sim.World) {
    if e.probe_cached && e.probe_id == e.selected && e.probe_time == world.time { return }
    e.probe, e.probe_available = replay.steering_probe(world, e.selected)
    e.probe_cached, e.probe_id, e.probe_time = true, e.selected, world.time
}

editor_draw_overlay :: proc(world: ^sim.World, e: Editor, camera: rl.Camera3D) {
    vehicle, found := editor_vehicle(world, e.selected)
    if !found { return }
    point, heading := sim.vehicle_pose(world, vehicle)
    position, forward := Vec3{point.x, 0, -point.y}, Vec3{heading.x, 0, -heading.y}
    editor_body_bounds(position+Vec3{0, 1.25, 0}, forward, vehicle.kind, MINT)
    tip := position+forward*3
    rl.DrawLine3D(position+Vec3{0, 0.5, 0}, tip+Vec3{0, 0.5, 0}, MINT)
    if vehicle.motion.active {
        steps := max(1, int(math.ceil((vehicle.motion.length-vehicle.motion.progress)/0.75)))
        p, _ := sim.connector_pose(vehicle.motion, vehicle.motion.progress)
        last := Vec3{p.x, 0.18, -p.y}
        for i in 1..=steps {
            distance := vehicle.motion.progress+(vehicle.motion.length-vehicle.motion.progress)*f32(i)/f32(steps)
            p, _ = sim.connector_pose(vehicle.motion, distance)
            next := Vec3{p.x, 0.18, -p.y}
            rl.DrawLine3D(last, next, rl.Color{255, 225, 95, 220})
            last = next
        }
    }
    if e.probe_available {
        last := position+Vec3{0, 0.3, 0}
        for sample in 1..=15 {
            p, _ := replay.probe_pose(e.probe, f32(sample)*0.05)
            next := Vec3{p.x, 0.3, -p.y}
            rl.DrawLine3D(last, next, rl.GREEN)
            last = next
        }
    }
    for other in world.vehicles {
        if !editor_is_blocker(world, vehicle, other) { continue }
        p, f := sim.vehicle_pose(world, other)
        editor_body_bounds(Vec3{p.x, 1.25, -p.y}, Vec3{f.x, 0, -f.y}, other.kind, rl.Color{255, 120, 90, 255})
    }
    _ = camera
}

editor_button :: proc(label: cstring, x, y, w: i32, active: bool) -> bool {
    rect := rl.Rectangle{f32(x), f32(y), f32(w), 27}
    hovered := rl.CheckCollisionPointRec(rl.GetMousePosition(), rect)
    color := rl.Color{49, 68, 75, 255}
    if active { color = rl.Color{54, 112, 91, 255} }
    if hovered { color = rl.Color{75, 112, 112, 255} }
    rl.DrawRectangleRec(rect, color)
    rl.DrawRectangleLinesEx(rect, 1, rl.Color{113, 145, 145, 255})
    rl.DrawText(label, x+8, y+6, 14, PAPER)
    return hovered && rl.IsMouseButtonPressed(.LEFT)
}

editor_panel :: proc(e: ^Editor, world: ^sim.World, previous: ^Render_Poses, accumulator: ^f64, paused: ^bool, camera: rl.Camera3D) -> Editor_Action {
    w, h := rl.GetScreenWidth(), rl.GetScreenHeight()
    x := w-EDITOR_WIDTH
    rl.DrawRectangle(x, 0, EDITOR_WIDTH, h, INK)
    rl.DrawRectangle(x, 0, 3, h, MINT)
    y := i32(12)
    rl.DrawText("SIMULATION EDITOR", x+14, y, 18, MINT); y += 29
    rl.DrawText(fmt.ctprintf("TIME %.2fs  /  %d vehicles", world.time, len(world.vehicles)), x+14, y, 14, PAPER); y += 25
    if editor_button("Pause / Play [Space]", x+12, y, EDITOR_WIDTH-24, !paused^) { paused^ = !paused^ }; y += 32
    if editor_button("Single 50ms tick", x+12, y, EDITOR_WIDTH-24, false) { paused^ = true; e.seeking = false; editor_tick(world, previous) }; y += 32
    if editor_button("Reset [R]", x+12, y, EDITOR_WIDTH-24, false) { if reset_world(world) { accumulator^ = 0; clear(previous); e.seeking = false; e.selected = 0 } }; y += 34
    rl.DrawText("Run to time (seconds)", x+14, y, 13, PAPER); y += 18
    rl.DrawRectangle(x+12, y, EDITOR_WIDTH-24, 27, rl.Color{35, 49, 55, 255})
    rl.DrawText(strings.clone_to_cstring(string(e.run_to_text[:e.run_to_len]), context.temp_allocator), x+20, y+6, 14, PAPER)
    rl.DrawRectangleLines(x+12, y, EDITOR_WIDTH-24, 27, MINT)
    if rl.CheckCollisionPointRec(rl.GetMousePosition(), rl.Rectangle{f32(x+12), f32(y), f32(EDITOR_WIDTH-24), 27}) && rl.IsMouseButtonPressed(.LEFT) { e.editing_time = true; e.run_to_len = 0; e.run_to_text[0] = 0 }
    if rl.IsKeyPressed(.ENTER) {
        e.editing_time = false
        value, ok := editor_parse_target(e)
        if ok { e.run_to = value } else { editor_set_result(e, "Run-to must be 0..3600 seconds") }
    }
    if e.editing_time {
        for { ch := rl.GetCharPressed(); if ch == 0 { break }; if e.run_to_len < len(e.run_to_text)-1 && ((ch >= '0' && ch <= '9') || ch == '.') { e.run_to_text[e.run_to_len] = u8(ch); e.run_to_len += 1; e.run_to_text[e.run_to_len] = 0 } }
        if rl.IsKeyPressed(.BACKSPACE) && e.run_to_len > 0 { e.run_to_len -= 1; e.run_to_text[e.run_to_len] = 0 }
    }
    y += 34
    if editor_button("Run deterministic seek", x+12, y, EDITOR_WIDTH-24, e.seeking) {
        target, ok := editor_parse_target(e)
        if !ok { editor_set_result(e, "Run-to must be 0..3600 seconds") }
        else {
            e.run_to = target
            reset_ok := true
            if target < world.time {
                reset_ok = reset_world(world)
                if reset_ok { accumulator^ = 0; clear(previous) }
            }
            if reset_ok { e.seeking = true; paused^ = true }
            else { editor_set_result(e, "Reset failed; seek cancelled") }
        }
    }; y += 32
    if editor_button("Save .cbor", x+12, y, (EDITOR_WIDTH-30)/3, false) { return .Save }
    if editor_button("Load .cbor", x+15+(EDITOR_WIDTH-30)/3, y, (EDITOR_WIDTH-30)/3, false) { return .Load }
    if editor_button("Check", x+18+2*(EDITOR_WIDTH-30)/3, y, (EDITOR_WIDTH-30)/3, false) { return .Check }; y += 34
    if e.result_len[0] > 0 { rl.DrawText(strings.clone_to_cstring(string(e.result[0][:e.result_len[0]]), context.temp_allocator), x+14, y, 12, rl.Color{255, 210, 130, 255}); y += 15 }
    if e.result_len[1] > 0 { rl.DrawText(strings.clone_to_cstring(string(e.result[1][:e.result_len[1]]), context.temp_allocator), x+14, y, 12, rl.Color{255, 210, 130, 255}); y += 15 }
    selected, found := editor_vehicle(world, e.selected)
    if found {
        road := selected.road
        exit := sim.successor(world, selected)
        if selected.motion.active { exit = selected.motion.exit }
        rl.DrawText(fmt.ctprintf("ID %d  %v / %v", selected.id, selected.kind, selected.driver.mode), x+14, y, 13, PAPER); y += 18
        rl.DrawText(fmt.ctprintf("Speed %.2f m/s | road %d -> %d", selected.speed, road, exit), x+14, y, 12, PAPER); y += 17
        index := 0
        for v, i in world.vehicles { if v.id == selected.id { index = i; break } }
        preferred: f32 = 2
        if world.behavior.enabled { preferred = selected.driver.traits.acceleration }
        following := sim.following_acceleration(world, selected, index, selected.driver.traits.headway, preferred)
        path_length := sim.road_length(world, road)
        if selected.motion.active { path_length = selected.motion.length }
        rl.DrawText(fmt.ctprintf("Signal %v | motion %.2f/%.1fm", sim.signal_state(world, road), selected.motion.progress, path_length), x+14, y, 12, PAPER); y += 17
        rl.DrawText(fmt.ctprintf("Following accel %.5f m/s2", following), x+14, y, 12, PAPER); y += 17
        if e.probe_available { rl.DrawText("Free 0.75m steering arc (green)", x+14, y, 12, rl.GREEN); y += 17 }
        rl.DrawText("Guide blockers (1m/s probe):", x+14, y, 12, MINT); y += 15
        count := 0
        for other in world.vehicles {
            if !editor_is_blocker(world, selected, other) { continue }
            rl.DrawText(fmt.ctprintf("#%d", other.id), x+24+i32(count%5*48), y+i32(count/5*14), 11, rl.Color{255, 150, 120, 255}); count += 1
        }
        y += 30
    } else { rl.DrawText("Click a vehicle or press Tab", x+14, y, 12, PAPER); y += 20 }
    rl.DrawText(fmt.ctprintf("roads %d | signals %d | spawns %d / done %d", len(world.roads), len(world.signals), world.spawned, world.completed), x+14, h-44, 12, MINT)
    rl.DrawText("Click driver / Tab cycle", x+14, h-24, 12, PAPER)
    _ = camera
    return .None
}

editor_set_result :: proc(e: ^Editor, message: string) {
    e.result_len[0] = min(len(message), len(e.result[0])-1)
    copy(e.result[0][:e.result_len[0]], message[:e.result_len[0]])
    e.result[0][e.result_len[0]] = 0
    e.result_len[1] = 0
}

editor_set_check_result :: proc(e: ^Editor, passed: bool, progress: f32, overlaps, off_pavement: int) {
    state := "FAIL"
    if passed { state = "PASS" }
    first := fmt.tprintf("%s progress %.2fm", state, progress)
    second := fmt.tprintf("overlap %d / off-road %d", overlaps, off_pavement)
    editor_set_result(e, first)
    e.result_len[1] = min(len(second), len(e.result[1])-1)
    copy(e.result[1][:e.result_len[1]], second[:e.result_len[1]])
    e.result[1][e.result_len[1]] = 0
}
