package showcase_test

import "core:testing"
import sim ".."
import mapdata "../../mapdata"

// Reproduce the two conflicting movements recorded at the showcase west cross.
// A full demand replay can change the queue order when driver behavior changes;
// this fixes the actual road geometry, green phase and the observed poses instead.
@(test)
test_showcase_cleared_turn_releases_straight_queue :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    testing.expect(t, ok); if !ok { return }
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    testing.expect(t, loaded); if !loaded { delete(nodes); delete(roads); return }
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    configured := sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers)
    delete(nodes); delete(roads); delete(signals); delete(sources)
    defer sim.destroy(&w)
    testing.expect(t, configured); if !configured { return }
    w.time = 93.95
    turning := sim.Vehicle{road = 6, distance = sim.entry_distance(&w, 6, .Motorcycle), lateral = -2.325,
        preferred_lateral = -2.325, speed = 4, kind = .Motorcycle, spawn_index = 1, id = 6,
        driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}}
    connector, problem := sim.make_connector(&w, turning, 25, -1.375)
    testing.expect(t, problem == .None); if problem != .None { return }
    connector.progress = 19.84
    turning.motion = connector
    straight := sim.Vehicle{road = 6, distance = sim.entry_distance(&w, 6, .Motorcycle), lateral = -0.465,
        preferred_lateral = -0.465, kind = .Motorcycle, spawn_index = 0, id = 12,
        driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}}
    append(&w.vehicles, turning, straight)
    testing.expect(t, sim.signal_state(&w, 6) == .Green)
    testing.expect(t, connector.progress < connector.lead+connector.arc[64])
    testing.expect(t, !sim.bodies_overlap(&w, turning, straight))
    sim.step(&w, 0.05)
    testing.expect(t, w.vehicles[1].motion.active && w.vehicles[0].motion.active &&
        w.vehicles[0].motion.progress < connector.lead+connector.arc[64],
        "straight rider must enter while the turning rider is still on its cleared curve")
    for _ in 0..<200 {
        sim.step(&w, 0.05)
        for a, i in w.vehicles { for b in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, a, b, 0)) } }
    }
}
