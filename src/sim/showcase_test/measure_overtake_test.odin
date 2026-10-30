package showcase_test

import "core:testing"
import sim ".."
import mapdata "../../mapdata"

@(test)
test_showcase_overtakes :: proc(t: ^testing.T) {
    for scale in ([]f32{1, 2}) {
        nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
        signals, sources, drivers, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
        assert(ok && loaded)
        for &s in sources { s.interval /= scale }
        w: sim.World
        sim.init(&w, nodes[:], roads[:], 1)
        assert(sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers))
        delete(nodes); delete(roads); delete(signals); delete(sources)
        started := make(map[u64]sim.Vehicle_Kind)
        previous := make(map[u64]bool)
        leaders := make(map[u64]u64)
        passed := make(map[u64]bool)
        for _ in 0..<2400 {
            sim.step(&w, 0.05)
            for a, i in w.vehicles { for b in w.vehicles[i+1:] { testing.expect(t, !sim.bodies_overlap(&w, a, b, 0)) } }
            testing.expect(t, w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
            for v in w.vehicles {
                if v.lane.active && !previous[v.id] {
                    started[v.id] = v.kind
                    gap := f32(100000)
                    for other in w.vehicles {
                        if other.id == v.id || other.road != v.road || other.distance <= v.distance { continue }
                        if abs(other.lateral-v.lane.from) >= (sim.kind_width(other.kind)+sim.kind_width(v.kind))*0.5+0.18 { continue }
                        if other.distance-v.distance < gap { gap = other.distance-v.distance; leaders[v.id] = other.id }
                    }
                }
                previous[v.id] = v.lane.active
                if leader, exists := leaders[v.id]; exists {
                    for other in w.vehicles { if other.id == leader && other.road == v.road && v.distance-other.distance > (sim.kind_length(v.kind)+sim.kind_length(other.kind))*0.5 { passed[v.id] = true } }
                }
            }
        }
        bike_starts, bike_passes := 0, 0
        for id, kind in started { if kind == .Motorcycle { bike_starts += 1; if passed[id] { bike_passes += 1 } } }
        testing.expect(t, bike_starts > 0 && bike_passes > 0, "showcase motorcycles must start and complete passes")
        delete(started); delete(previous); delete(leaders); delete(passed); sim.destroy(&w)
    }
}
