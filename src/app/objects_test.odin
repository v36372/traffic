package app

import "core:fmt"
import "core:math"
import "core:os"
import "core:testing"
import sim "../sim"

@(test)
test_objects_transaction_and_optional_absence :: proc(t: ^testing.T) {
    dir, err := os.make_directory_temp("", "traffic-app-objects-*", context.allocator)
    testing.expect(t, err == nil)
    if err != nil { return }
    defer delete(dir)
    defer os.remove(dir)
    path := fmt.aprintf("%s/map.csv", dir)
    defer delete(path)
    defer os.remove(path)
    sidecar := objects_sidecar(path)
    defer delete(sidecar)
    defer os.remove(sidecar)
    testing.expect(t, os.write_entire_file(path, "HCM_MAP,1\nN,0,0\nN,45,0\nN,85,0\nN,45,30\nE,0,1,7,9\nE,1,2,7,9\nE,3,1,5,7\n") == nil)
    world: sim.World
    sim.init_demo(&world)
    defer sim.destroy(&world)
    // Truly missing objects are valid for older authored maps and OSM data.
    testing.expect(t, load_map(&world, path))
    testing.expect(t, len(world.signals) == 0 && len(world.spawns) == 0)
    valid := "TRAFFIC_OBJECTS,1\nS,1,2,1,1,0\nP,0,2,0.8,0\nD,0,0.6,0.9,0.5,1.5\n"
    testing.expect(t, os.write_entire_file(sidecar, valid) == nil)
    testing.expect(t, load_map(&world, path))
    testing.expect(t, len(world.signals) == 1 && len(world.spawns) == 1 && world.behavior.enabled && world.behavior.classes[0].speed_min == 0.6)
    testing.expect(t, reset_world(&world) && world.behavior.enabled && world.behavior.classes[0].headway_max == 1.5)
    for _ in 0..<100 { sim.step(&world, f32(TICK)) }
    clock, rng, spawned, pending := world.time, world.rng, world.spawned, world.pending
    nodes, roads, vehicles := raw_data(world.nodes), raw_data(world.roads), raw_data(world.vehicles)
    signals, spawns := raw_data(world.signals), raw_data(world.spawns)
    active := len(world.vehicles)
    // Both malformed structure and semantically invalid topology must preserve
    // the complete live world, not silently load geometry without its controls.
    for invalid in ([]string{"", "TRAFFIC_OBJECTS,9\n", "TRAFFIC_OBJECTS,1\nS,1,nan,1,1,0\n", "TRAFFIC_OBJECTS,1\nS,0,2,1,1,0\n", "TRAFFIC_OBJECTS,1\nP,0,3,0.8,0\n", "TRAFFIC_OBJECTS,1\nD,0,0.5,1.1,0.4,2\n"}) {
        testing.expect(t, os.write_entire_file(sidecar, invalid) == nil)
        testing.expect(t, !load_map(&world, path))
        testing.expect(t, raw_data(world.nodes) == nodes && raw_data(world.roads) == roads)
        testing.expect(t, raw_data(world.vehicles) == vehicles && len(world.vehicles) == active)
        testing.expect(t, raw_data(world.signals) == signals && raw_data(world.spawns) == spawns)
        testing.expect(t, world.time == clock && world.rng == rng && world.spawned == spawned && world.pending == pending && world.behavior.enabled)
    }
    // A present path that isn't readable as an objects file is NOT optional.
    testing.expect(t, os.remove(sidecar) == nil)
    testing.expect(t, os.make_directory(sidecar) == nil)
    testing.expect(t, !load_map(&world, path))
    testing.expect(t, world.time == clock && raw_data(world.signals) == signals)
    testing.expect(t, os.remove(sidecar) == nil)
    // Switching to an object-free map clears prior objects and demand counters.
    testing.expect(t, load_map(&world, path))
    testing.expect(t, len(world.signals) == 0 && len(world.spawns) == 0 && world.time == 0 && world.pending == 0)
}

expect_same_traffic :: proc(t: ^testing.T, a, b: ^sim.World) {
    testing.expect(t, a.time == b.time && a.rng == b.rng && a.pending == b.pending && a.requested == b.requested)
    testing.expect(t, a.spawned == b.spawned && a.completed == b.completed && len(a.vehicles) == len(b.vehicles))
    if len(a.vehicles) == len(b.vehicles) {
        for vehicle, i in a.vehicles { testing.expect(t, vehicle == b.vehicles[i]) }
    }
}

@(test)
test_hcm_map_enables_motorcycle_preferences :: proc(t: ^testing.T) {
    world: sim.World
    sim.init_demo(&world)
    defer sim.destroy(&world)
    testing.expect(t, load_map(&world, "data/hcm.csv"))
    testing.expect(t, world.behavior.enabled && len(world.spawns) == 0 && len(world.signals) == 0)
    riders := world.behavior.classes[int(sim.Vehicle_Kind.Motorcycle)]
    testing.expect(t, riders.speed_min == 0.65 && riders.speed_max == 1 && riders.headway_min == 0.5 && riders.headway_max == 1.5)
}

@(test)
test_reset_preserves_configuration_restarts_clocks_and_rng :: proc(t: ^testing.T) {
    world, reference: sim.World
    sim.init_demo(&world)
    sim.init_demo(&reference)
    defer sim.destroy(&world)
    defer sim.destroy(&reference)
    plans := []sim.Signal_Plan{{1, 2, 1, 1, 0.5, false}}
    sources := []sim.Spawn_Point{{0, 2, 0.6, .Motorcycle}, {2, 2, 1.2, .Car}}
    testing.expect(t, sim.configure(&world, plans, sources))
    testing.expect(t, sim.configure(&reference, plans, sources))
    for _ in 0..<200 { sim.step(&world, f32(TICK)) }
    testing.expect(t, world.time > 0 && world.requested > 0)
    testing.expect(t, reset_world(&world))
    testing.expect(t, world.time == 0 && world.spawned == 0 && world.completed == 0 && world.pending == 0 && world.requested == 0)
    testing.expect(t, len(world.signals) == 1 && world.signals[0] == plans[0])
    testing.expect(t, len(world.spawns) == 2 && world.spawns[0] == sources[0] && world.spawns[1] == sources[1])
    testing.expect(t, sim.signal_state(&world, 0) == sim.signal_state(&reference, 0))
    for _ in 0..<200 { sim.step(&world, f32(TICK)); sim.step(&reference, f32(TICK)) }
    expect_same_traffic(t, &world, &reference)
    // Synthetic mode also restarts its seeded RNG, not merely the clock.
    sim.destroy(&world); sim.init_demo(&world)
    sim.destroy(&reference); sim.init_demo(&reference)
    for _ in 0..<200 { sim.step(&world, f32(TICK)) }
    testing.expect(t, reset_world(&world))
    for _ in 0..<200 { sim.step(&world, f32(TICK)); sim.step(&reference, f32(TICK)) }
    expect_same_traffic(t, &world, &reference)
}

@(test)
test_signal_approach_model_geometry_and_phase_colors :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{20, 0}}, {{20, 20}}, {{40, 0}}},
        []sim.Road{{0, 1, 6, 9}, {2, 1, 5, 9}, {1, 2, 5, 9}, {1, 3, 6, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    uncontrolled := signal_approaches(&world, offsets)
    defer delete(uncontrolled)
    testing.expect(t, len(uncontrolled) == 0, "no invented controls on object-free maps")
    testing.expect(t, sim.configure(&world, []sim.Signal_Plan{{1, 2, 1, 1, 0, false}}, nil))
    approaches := signal_approaches(&world, offsets)
    defer delete(approaches)
    testing.expect(t, len(approaches) == 2)
    if len(approaches) != 2 { return }
    testing.expect(t, approaches[0].road == 0 && approaches[1].road == 1)
    pavement := build_road_scene(&world, offsets)
    defer destroy_road_scene(&pavement)
    for approach in approaches {
        road := world.roads[approach.road]
        start, forward, left, _ := road_frame(&world, road)
        center := start+forward*sim.stop_line_distance(&world, approach.road)+left*sim.road_offset(&world, approach.road)
        testing.expect(t, math.abs(approach.stop_center[0]-center[0]) < 0.001 && math.abs(approach.stop_center[2]-center[2]) < 0.001)
        testing.expect(t, approach.stop_center[1] == 0.055 && approach.stop_width == road.width)
        testing.expect(t, !road_point_covered(&pavement, 2, approach.pole), "pole must be outside ALL asphalt, not only its own carriageway")
        testing.expect(t, approach.forward == forward && approach.left == left)
    }
    // Color selection accepts the simulation enum directly, without a render clock.
    red := signal_lamp_color(.Red, 0)
    amber := signal_lamp_color(.Amber, 1)
    green := signal_lamp_color(.Green, 2)
    testing.expect(t, red.r >= 240 && red.g < 40 && red.b < 40, "red lamp must read as red")
    testing.expect(t, amber.r >= 240 && amber.g >= 160 && amber.b < 40, "amber lamp must read as yellow")
    testing.expect(t, green.g >= 200 && green.r < 40 && green.b < 120, "green lamp must read as green")
    testing.expect(t, signal_state_color(.Red) == red && signal_state_color(.Amber) == amber && signal_state_color(.Green) == green)
    testing.expect(t, signal_lamp_color(.Green, 0) != signal_lamp_color(.Red, 0))
    testing.expect(t, signal_lamp_color(.Uncontrolled, 0) != signal_state_color(.Red))
    testing.expect(t, signal_model(sim.Signal_Plan{}) == .Three_Aspect)
    testing.expect(t, signal_model(sim.Signal_Plan{countdown = true}) == .Countdown)
    testing.expect(t, seven_segment_mask(8) == 0x7F && seven_segment_mask(1) == 0x06 && seven_segment_mask(10) == 0)
    testing.expect(t, signal_countdown_places(9) == 2 && signal_countdown_places(99) == 2 && signal_countdown_places(100) == 3)
    for phase, i in ([]sim.Signal_State{.Green, .Amber, .Red}) {
        world.time = f64(i)+1
        state := sim.signal_state(&world, 0)
        testing.expect(t, state == phase)
        for lamp in 0..<3 { testing.expect(t, signal_lamp_color(state, lamp) == signal_lamp_color(phase, lamp)) }
    }
}
