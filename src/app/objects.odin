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
        signals, spawns, objects_ok := mapdata.load_objects(sidecar)
        defer delete(signals)
        defer delete(spawns)
        // Scale only the candidate configuration, before clocks start. configure
        // enforces timer bounds; reset copies these adjusted intervals unchanged.
        for &source in spawns { source.interval /= demand_scale }
        if !objects_ok || !sim.configure(&candidate, signals[:], spawns[:]) {
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
    if !sim.configure(&candidate, world.signals[:], world.spawns[:]) { return false }
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

signal_lamp_color :: proc(state: sim.Signal_State, lamp: int) -> rl.Color {
    if state == .Red && lamp == 0 { return {255, 71, 61, 255} }
    if state == .Amber && lamp == 1 { return {255, 192, 51, 255} }
    if state == .Green && lamp == 2 { return {75, 247, 137, 255} }
    return {47, 57, 59, 255}
}

draw_signals :: proc(world: ^sim.World, approaches: []Signal_Approach) {
    // Exclusive-approach signals visualize simulation control, not a render clock.
    for approach in approaches {
        state := sim.signal_state(world, approach.road)
        strip(approach.stop_center-approach.forward*0.125, approach.stop_center+approach.forward*0.125, approach.left, approach.stop_width, PAPER)
        gl.PushMatrix()
        gl.Translatef(approach.pole[0], 0, approach.pole[2])
        gl.Rotatef(math.atan2(approach.forward[0], approach.forward[2])*180/math.PI, 0, 1, 0)
        // Meter-scale poles and compact 0.4m lenses; no zoom-scaled props.
        // Heads face approaching traffic (-local Z), never a camera-facing overlay.
        rl.DrawCylinderEx({0, 0.05, 0}, {0, 4.1, 0}, 0.075, 0.075, 8, GLASS)
        rl.DrawCubeV({0, 0.1, 0}, {0.32, 0.2, 0.32}, RUBBER)
        rl.DrawCubeV({0, 3.65, -0.12}, {0.62, 1.55, 0.36}, RUBBER)
        rl.DrawCubeV({0, 4.46, -0.19}, {0.76, 0.10, 0.57}, GLASS)
        for lamp in 0..<3 {
            y := f32(4.14)-f32(lamp)*0.49
            color := signal_lamp_color(state, lamp)
            rl.DrawCylinderEx({0, y, -0.31}, {0, y, -0.39}, 0.20, 0.20, 12, color)
        }
        gl.PopMatrix()
    }
}
