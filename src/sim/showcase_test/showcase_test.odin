package showcase_test

import "core:testing"
import "core:fmt"
import sim ".."
import mapdata "../../mapdata"

// Run from the repository root after `make maps`:
// odin test src/sim/showcase_test
@(test)
test_busy_showcase_preserves_displaced_motorcycle_corridors :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded)
    if !loaded { return }
    defer delete(signals)
    defer delete(sources)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]))
    testing.expect(t, sim.configure_drivers(&w, drivers))
    // Captured ID7 was displaced from its preferred band on road 6: its
    // world Y was -5.075, but its old receiving target was -4.125.
    append(&w.vehicles, sim.Vehicle{road = 6, distance = sim.entry_distance(&w, 6, .Motorcycle), lateral = -2.325,
        preferred_lateral = -1.375, kind = .Motorcycle, spawn_index = 0, id = 7})
    append(&w.vehicles, sim.Vehicle{road = 6, distance = sim.entry_distance(&w, 6, .Motorcycle), lateral = -0.8,
        preferred_lateral = 0, kind = .Motorcycle, spawn_index = 0, id = 12})
    testing.expect(t, sim.successor(&w, w.vehicles[0]) == 8)
    old, problem := sim.make_connector(&w, w.vehicles[0], 8, -1.375)
    _, old_straight := sim.straight_direction(old)
    testing.expect(t, problem == .None && !old_straight, "captured preferred target converges and cannot batch")
    testing.expect(t, sim.acquire_connector(&w, 0, 8))
    testing.expect(t, sim.acquire_connector(&w, 1, 8))
    a, b := w.vehicles[0], w.vehicles[1]
    testing.expect(t, abs(a.motion.lateral-a.lateral) < 0.00001 && abs(b.motion.lateral-b.lateral) < 0.00001)
    testing.expect(t, sim.straight_corridors_disjoint(a.motion, a.kind, b.motion, b.kind))
    hull := sim.junction_polygon(&w, w.roads[6].to)
    defer delete(hull)
    testing.expect(t, sim.connector_on_pavement(&w, a.motion, a.kind, hull[:]))
    testing.expect(t, sim.connector_on_pavement(&w, b.motion, b.kind, hull[:]))
    for _ in 0..<300 {
        sim.move_vehicle(&w, 0, 0.1)
        sim.move_vehicle(&w, 1, 0.1)
        testing.expect(t, !sim.bodies_overlap(&w, w.vehicles[0], w.vehicles[1], 0))
    }
}

@(test)
test_busy_showcase_capture_window :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded)
    if !loaded { return }
    defer delete(signals)
    defer delete(sources)
    for &source in sources { source.interval /= 2 }
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]))
    testing.expect(t, sim.configure_drivers(&w, drivers))
    max_owners := 0
    max_turns, moving_turn_ticks := 0, 0
    previous := make(map[u64]f32)
    defer delete(previous)
    for tick in 0..<1940 {
        sim.step(&w, 0.05)
        owners, turns, moving_turns := 0, 0, 0
        for a, i in w.vehicles {
            if a.motion.active && a.motion.node == w.roads[6].to { owners += 1 }
            if tick >= 1760 && tick < 1920 && a.road == 6 && a.motion.active && a.motion.exit == 25 {
                turns += 1
                if old, found := previous[a.id]; found && a.motion.progress > old+0.001 { moving_turns += 1 }
            }
            if a.motion.active { previous[a.id] = a.motion.progress }
            for b in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, a, b, 0)) }
        }
        if tick >= 1760 && tick < 1920 { max_turns = max(max_turns, turns); if moving_turns >= 2 { moving_turn_ticks += 1 } }
        testing.expect(t, w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
        if tick >= 1740 { max_owners = max(max_owners, owners) }
    }
    testing.expect(t, max_owners >= 2, "busy capture window must demonstrate parallel discharge at the focused junction")
    testing.expect(t, moving_turn_ticks > 0, fmt.tprintf("88-96s replay: max concurrent 6->25 turns=%d ticks with multiple moving=%d", max_turns, moving_turn_ticks))
}

@(test)
test_showcase_truck_clears_uncontrolled_bend :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded)
    if !loaded { return }
    defer delete(signals)
    defer delete(sources)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]))
    testing.expect(t, sim.configure_drivers(&w, drivers))
    saw_turn, cleared := false, false
    for _ in 0..<2400 {
        sim.step(&w, 0.05)
        for v, i in w.vehicles {
            for other in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, v, other, 0)) }
            if v.id != 26 { continue }
            if v.road == 19 && v.motion.active && v.motion.exit == 0 {
                if !saw_turn {
                    saw_turn = true
                    testing.expect(t, v.motion.lead < sim.turn_start_distance(&w, 19)-sim.entry_distance(&w, 19, v.kind),
                        "truck must start steering before the mouth after reserving the uncontrolled entry")
                    hull := sim.junction_polygon(&w, v.motion.node)
                    testing.expect(t, sim.connector_on_pavement(&w, v.motion, v.kind, hull[:]))
                    delete(hull)
                }
                testing.expect(t, v.motion.progress < v.motion.length)
            }
            if saw_turn && v.road == 0 { cleared = true }
        }
        testing.expect(t, w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
    }
    testing.expect(t, saw_turn && cleared, "actual seeded truck must negotiate 19->0 and release the node")
}

@(test)
test_controlled_showcase_bend_never_steers_before_mouth :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded)
    if !loaded { return }
    defer delete(signals)
    defer delete(sources)
    append(&signals, sim.Signal_Plan{0,100,2,1,0})
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    configured := sim.configure(&w, signals[:], sources[:])
    testing.expect(t, configured && sim.configure_drivers(&w, drivers))
    if !configured { return }
    v := sim.Vehicle{road = 19, distance = sim.entry_distance(&w, 19, .Truck), kind = .Truck, spawn_index = 9}
    testing.expect(t, sim.signal_state(&w, 19) != .Uncontrolled)
    c, problem := sim.make_connector(&w, v, 0, 0)
    if problem == .None {
        testing.expect(t, c.lead >= sim.turn_start_distance(&w, 19)-v.distance,
            "controlled crossings must retain the straight lead")
    } else {
        testing.expect(t, problem == .Tight_Turn)
    }
}

@(test)
test_showcase_motorcycles_vary_and_respond_to_traffic :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded && drivers.enabled)
    if !loaded { return }
    defer delete(signals)
    defer delete(sources)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    defer sim.destroy(&w)
    testing.expect(t, sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers))
    previous := make(map[u64]f32)
    defer delete(previous)
    min_factor, max_factor: f32 = 1, 0
    saw_acceleration, saw_braking, saw_different_speeds := false, false, false
    for _ in 0..<800 {
        sim.step(&w, 0.05)
        min_speed, max_speed: f32 = 100, 0
        for v in w.vehicles {
            if v.kind != .Motorcycle { continue }
            min_factor = min(min_factor, v.driver.traits.speed_factor)
            max_factor = max(max_factor, v.driver.traits.speed_factor)
            if v.speed > 1 { min_speed, max_speed = min(min_speed, v.speed), max(max_speed, v.speed) }
            if old, found := previous[v.id]; found {
                if v.speed > old+0.01 { saw_acceleration = true }
                if v.speed < old-0.01 { saw_braking = true }
            }
            previous[v.id] = v.speed
        }
        if min_speed < 100 && max_speed-min_speed > 0.5 { saw_different_speeds = true }
    }
    testing.expect(t, max_factor-min_factor > 0.1, "showcase must enable distinct persistent rider preferences")
    testing.expect(t, saw_different_speeds, "motorcycles in the live scene must not all travel at the same speed")
    testing.expect(t, saw_acceleration && saw_braking, "riders must change speed in response to traffic")
}
