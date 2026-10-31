package showcase_test

import "core:testing"
import sim ".."
import mapdata "../../mapdata"

// The default seed used to leave rider 41 stopped mid-pass from 60s to 160s,
// with rider 47 held behind it. Exercise the real unmodified startup and queue.
@(test)
test_showcase_stopped_follower_does_not_freeze_passing_rider :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
    signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
    assert(ok && loaded)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 1)
    assert(sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers))
    delete(nodes); delete(roads); delete(signals); delete(sources)
    defer sim.destroy(&w)
    seen, present_at_60, cleared := false, false, false
    start_road := -1
    start_distance: f32
    // The local-gap model finishes this pass before 60s; rider 41 then waits
    // at a real red light through 75s. Test pass completion separately from
    // resuming after the next green, rather than requiring red-light motion.
    for tick in 1..=2400 {
        sim.step(&w, 0.05)
        found := false
        for v, i in w.vehicles {
            if tick >= 1200 { for other in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, v, other, 0)) } }
            if v.id != 41 { continue }
            seen, found = true, true
            if tick == 1200 {
                present_at_60, start_road, start_distance = true, v.road, v.distance
                testing.expect(t, !v.lane.active, "rider 41 must finish passing before joining the signal queue")
            }
            if tick > 1200 && present_at_60 && (v.road != start_road || v.distance > start_distance+0.5) { cleared = true }
        }
        if tick >= 1200 && seen && !found { cleared = true }
        testing.expect(t, w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
    }
    testing.expect(t, seen && cleared, "default-seed rider 41 must finish passing and resume after the next green")
}
