package app

import "core:fmt"
import "core:math"
import "core:os"
import "core:path/filepath"
import "core:strings"
import mapdata "../mapdata"
import sim "../sim"
import rl "vendor:raylib"
import gl "vendor:raylib/rlgl"

objects_sidecar :: proc(path: string) -> string {
    return fmt.aprintf("%s-objects.csv", strings.trim_suffix(path, filepath.ext(path)))
}

load_map :: proc(world: ^sim.World, path: string, demand_scale: f32 = 1) -> bool {
    if !sim.finite(demand_scale) || demand_scale <= 0 || demand_scale > 20 {
        fmt.eprintln("Demand scale must be finite, >0 and <=20; retaining prior scene")
        return false
    }
    nodes, roads, ok := mapdata.load(path)
    defer delete(nodes)
    defer delete(roads)
    if !ok { fmt.eprintf("Map load failed: %s; retaining prior scene\n", path); return false }
    candidate: sim.World
    sim.init(&candidate, nodes[:], roads[:])
    // After the ownership swap this destroys the old world; on failure, only the candidate.
    defer sim.destroy(&candidate)
    sidecar := objects_sidecar(path)
    defer delete(sidecar)
    // lstat distinguishes genuinely missing optional files from unreadable files,
    // directories and broken symlinks; none of those may silently discard controls.
    info, err := os.lstat(sidecar, context.allocator)
    if err == nil { os.file_info_delete(info, context.allocator) }
    if err != .Not_Exist {
        if err != nil { fmt.eprintf("Objects unavailable: %s; retaining prior scene\n", sidecar); return false }
        signals, spawns, drivers, objects_ok := mapdata.load_objects(sidecar)
        defer delete(signals)
        defer delete(spawns)
        // Scale only the candidate configuration, before clocks start. configure
        // enforces timer bounds; reset copies these adjusted intervals unchanged.
        for &source in spawns { source.interval /= demand_scale }
        if !objects_ok || !sim.configure(&candidate, signals[:], spawns[:]) || !sim.configure_drivers(&candidate, drivers) {
            fmt.eprintf("Objects invalid: %s; retaining prior scene\n", sidecar)
            return false
        }
    }
    if demand_scale != 1 && len(candidate.spawns) == 0 {
        fmt.eprintln("--demand-scale requires configured sources; retaining prior scene")
        return false
    }
    world^, candidate = candidate, world^
    fmt.printf("Map loaded: %s (%d nodes, %d directed roads, %d lights, %d sources)\n", path, len(world.nodes), len(world.roads), len(world.signals), len(world.spawns))
    fmt.printf("Synthetic demand scale: %.4gx (not measured traffic counts)\n", demand_scale)
    return true
}

// Frontend worlds always start with seed 1. Build the replacement while the
// original's owned geometry/configuration is still alive; configure copies both.
reset_world :: proc(world: ^sim.World) -> bool {
    candidate: sim.World
    sim.init(&candidate, world.nodes[:], world.roads[:])
    defer sim.destroy(&candidate)
    if !sim.configure(&candidate, world.signals[:], world.spawns[:]) || !sim.configure_drivers(&candidate, world.behavior) { return false }
    world^, candidate = candidate, world^
    return true
}

Signal_Approach :: struct {
    road: int,
    stop_center, pole, forward, left: Vec3,
    stop_width: f32,
}

// Cache only controlled approaches once per map switch, never all-pairs per frame.
signal_approaches :: proc(world: ^sim.World, offsets: []f32) -> [dynamic]Signal_Approach {
    controlled := make(map[int]bool)
    defer delete(controlled)
    for plan in world.signals { controlled[plan.node] = true }
    approaches: [dynamic]Signal_Approach
    for road, i in world.roads {
        if !controlled[road.to] { continue }
        start, forward, left, length := road_frame(world, road)
        if length <= 0 { continue }
        // Shared FRONT limit includes the junction hull, crossing strips and
        // roadside-pole clearance. A node is the junction center, not its entry.
        stop := start+forward*sim.stop_line_distance(world, i)+left*offsets[i]
        pole := stop-left*(road.width*0.5+0.45)
        append(&approaches, Signal_Approach{i, stop+Vec3{0, 0.055, 0}, pole, forward, left, road.width})
    }
    return approaches
}

// Head is tilted so the lamp faces read from the overhead camera while still
// pointing at approaching traffic. Pole base size is a clearance contract.
SIGNAL_HEAD_TILT :: 52
SIGNAL_HEAD_HEIGHT :: 4.35
SIGNAL_LAMP_SPAN :: 0.56
SEVEN_SEGMENT := [10]u8{0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F}

Signal_Model :: enum { Three_Aspect, Countdown }

signal_model :: proc(plan: sim.Signal_Plan) -> Signal_Model {
    if plan.countdown { return .Countdown }
    return .Three_Aspect
}

seven_segment_mask :: proc(digit: int) -> u8 {
    if digit < 0 || digit > 9 { return 0 }
    return SEVEN_SEGMENT[digit]
}

signal_countdown_places :: proc(seconds: int) -> int {
    if seconds >= 100 { return 3 }
    return 2
}

signal_state_color :: proc(state: sim.Signal_State) -> rl.Color {
    switch state {
    case .Red: return {255, 24, 24, 255}
    case .Amber: return {255, 214, 0, 255}
    case .Green: return {16, 230, 56, 255}
    case .Uncontrolled: return {22, 24, 26, 255}
    }
    return {22, 24, 26, 255}
}

signal_lamp_color :: proc(state: sim.Signal_State, lamp: int) -> rl.Color {
    if state == .Red && lamp == 0 { return signal_state_color(.Red) }
    if state == .Amber && lamp == 1 { return signal_state_color(.Amber) }
    if state == .Green && lamp == 2 { return signal_state_color(.Green) }
    switch lamp {
    case 0: return {48, 14, 14, 255}
    case 1: return {48, 36, 10, 255}
    case 2: return {12, 44, 20, 255}
    }
    return {22, 24, 26, 255}
}

draw_seven_segment :: proc(center: Vec3, digit: int, color: rl.Color, w, h, t: f32) {
    mask := seven_segment_mask(digit)
    hw, hh := w*0.5, h*0.5
    x, y, z := center.x, center.y, center.z
    if mask & 0x01 != 0 { rl.DrawCubeV({x, y+hh, z}, {w, t, t}, color) }
    if mask & 0x02 != 0 { rl.DrawCubeV({x+hw, y+hh*0.5, z}, {t, h*0.42, t}, color) }
    if mask & 0x04 != 0 { rl.DrawCubeV({x+hw, y-hh*0.5, z}, {t, h*0.42, t}, color) }
    if mask & 0x08 != 0 { rl.DrawCubeV({x, y-hh, z}, {w, t, t}, color) }
    if mask & 0x10 != 0 { rl.DrawCubeV({x-hw, y-hh*0.5, z}, {t, h*0.42, t}, color) }
    if mask & 0x20 != 0 { rl.DrawCubeV({x-hw, y+hh*0.5, z}, {t, h*0.42, t}, color) }
    if mask & 0x40 != 0 { rl.DrawCubeV({x, y, z}, {w, t, t}, color) }
}

draw_signal_lamp :: proc(y: f32, lamp: int, state: sim.Signal_State) {
    color := signal_lamp_color(state, lamp)
    active := (lamp == 0 && state == .Red) || (lamp == 1 && state == .Amber) || (lamp == 2 && state == .Green)
    rl.DrawCubeV({0, y+0.2, -0.28}, {0.54, 0.07, 0.32}, RUBBER)
    rl.DrawCylinderEx({0, y, -0.12}, {0, y, -0.34}, 0.24, 0.24, 16, {18, 20, 22, 255})
    rl.DrawCylinderEx({0, y, -0.34}, {0, y, -0.42}, active ? 0.23 : 0.2, active ? 0.23 : 0.2, 16, color)
    rl.DrawSphere({0, y, -0.4}, active ? 0.22 : 0.16, color)
    if active {
        rl.DrawSphere({0, y, -0.4}, 0.34, {color.r, color.g, color.b, 70})
    }
}

draw_countdown_module :: proc(y: f32, state: sim.Signal_State, seconds: int) {
    places := signal_countdown_places(seconds)
    box_w: f32 = places == 3 ? 0.96 : 0.7
    lit := signal_state_color(state)
    dim := rl.Color{u8(int(lit.r)*2/5), u8(int(lit.g)*2/5), u8(int(lit.b)*2/5), 255}
    rl.DrawCubeV({0, y, 0.02}, {box_w, 0.62, 0.3}, RUBBER)
    rl.DrawCubeV({0, y, -0.14}, {box_w-0.08, 0.5, 0.05}, dim)
    value := seconds
    if value < 0 { value = 0 }
    if places == 3 && value > 999 { value = 999 }
    if places == 2 && value > 99 { value = 99 }
    digit_w: f32 = 0.2
    gap: f32 = 0.07
    total := f32(places)*digit_w + f32(places-1)*gap
    start := -total*0.5 + digit_w*0.5
    for i in 0..<places {
        place := places-1-i
        d := value
        for _ in 0..<place { d /= 10 }
        d %= 10
        draw_seven_segment({start+f32(i)*(digit_w+gap), y, -0.2}, d, lit, digit_w, 0.38, 0.06)
    }
}

draw_signal_head :: proc(state: sim.Signal_State, model: Signal_Model, seconds: int) {
    top: f32 = 0.82
    if model == .Countdown {
        draw_countdown_module(top, state, seconds)
        top -= 0.58
    }
    housing_h: f32 = 3*SIGNAL_LAMP_SPAN + 0.22
    housing_y := top - housing_h*0.5 + 0.08
    rl.DrawCubeV({0, housing_y, 0.04}, {0.62, housing_h, 0.38}, RUBBER)
    for lamp in 0..<3 {
        draw_signal_lamp(top-0.18-f32(lamp)*SIGNAL_LAMP_SPAN, lamp, state)
    }
}

draw_signals :: proc(world: ^sim.World, approaches: []Signal_Approach) {
    // Exclusive-approach signals visualize simulation control, not a render clock.
    for approach in approaches {
        state, remaining, plan, ok := sim.signal_phase(world, approach.road)
        strip(approach.stop_center-approach.forward*0.125, approach.stop_center+approach.forward*0.125, approach.left, approach.stop_width, PAPER)
        if !ok || state == .Uncontrolled { continue }
        gl.PushMatrix()
        gl.Translatef(approach.pole[0], 0, approach.pole[2])
        gl.Rotatef(math.atan2(approach.forward[0], approach.forward[2])*180/math.PI, 0, 1, 0)
        rl.DrawCylinderEx({0, 0.05, 0}, {0, SIGNAL_HEAD_HEIGHT, 0}, 0.08, 0.08, 8, GLASS)
        rl.DrawCubeV({0, 0.1, 0}, {0.32, 0.2, 0.32}, RUBBER)
        rl.DrawCubeV({0, SIGNAL_HEAD_HEIGHT-0.15, 0.02}, {0.22, 0.16, 0.28}, RUBBER)
        gl.Translatef(0, SIGNAL_HEAD_HEIGHT, -0.18)
        gl.Rotatef(SIGNAL_HEAD_TILT, 1, 0, 0)
        seconds := sim.signal_countdown_seconds(remaining)
        draw_signal_head(state, signal_model(plan), seconds)
        gl.PopMatrix()
    }
}
