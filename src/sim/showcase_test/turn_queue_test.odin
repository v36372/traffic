package showcase_test

import "core:testing"
import "core:fmt"
import sim ".."
import mapdata "../../mapdata"

@(test)
test_showcase_different_lateral_turn_queue_five_seconds :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok); if !ok { return }
    defer delete(nodes); defer delete(roads)
    signals, sources, _, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded); if !loaded { return }
    defer delete(signals); defer delete(sources)
    for &s in sources { s.interval = 86400 }
    for &s in signals { if s.node == roads[6].to { s.green, s.offset = 100, 0 } }
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]))
    entry := sim.entry_distance(&w, 6, .Motorcycle)
    append(&w.vehicles,
        sim.Vehicle{road = 6, distance = entry, lateral = -2.325, preferred_lateral = -1.375, kind = .Motorcycle, spawn_index = 1, id = 1},
        sim.Vehicle{road = 6, distance = entry, lateral = -0.465, preferred_lateral = -1.375, kind = .Motorcycle, spawn_index = 1, id = 2})
    testing.expect(t, sim.successor(&w, w.vehicles[0]) == 25 && sim.signal_state(&w, 6) == .Green)
    w.spawned, w.requested = 2, 2
    concurrently, leader_moved, follower_moved := false, false, false
    previous: [2]f32
    for _ in 0..<100 {
        sim.step(&w, 0.05)
        for a, i in w.vehicles { for b in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, a, b, 0)) } }
        if len(w.vehicles) == 2 && w.vehicles[0].motion.active && w.vehicles[1].motion.active {
            concurrently = true
            if w.vehicles[0].motion.progress > previous[0]+0.001 { leader_moved = true }
            if w.vehicles[1].motion.progress > previous[1]+0.001 { follower_moved = true }
        }
        for v in w.vehicles { if v.id <= 2 && v.motion.active { previous[int(v.id)-1] = v.motion.progress } }
    }
    testing.expect(t, concurrently && leader_moved && follower_moved,
        fmt.tprintf("turn queue concurrent=%v leader=%v follower=%v; vehicles=%v", concurrently, leader_moved, follower_moved, w.vehicles[:]))
}

@(test)
test_default_demand_turn_queue_replay_88_to_96 :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok); if !ok { return }
    defer delete(nodes); defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded); if !loaded { return }
    defer delete(signals); defer delete(sources)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers))
    previous := make(map[u64]f32)
    defer delete(previous)
    multiple_moving := 0
    for tick in 0..<1920 {
        sim.step(&w, 0.05)
        moving := 0
        for v, i in w.vehicles {
            for other in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, v, other, 0)) }
            if tick >= 1760 && v.road == 6 && v.motion.active && v.motion.exit == 25 {
                if old, found := previous[v.id]; found && v.motion.progress > old+0.001 { moving += 1 }
                previous[v.id] = v.motion.progress
            }
        }
        if moving >= 2 { multiple_moving += 1 }
    }
    testing.expect(t, multiple_moving > 0, "default seeded 88-96s replay must move multiple 6->25 riders together")
}
