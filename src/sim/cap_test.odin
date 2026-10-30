package sim

import "core:testing"

@(test)
test_pending_at_cap_and_recovery :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{1000,0}}}, []Road{{0,1,3,0}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,1,0.05,.Car}}))
    // Populate a cap-sized stationary queue representing already admitted demand.
    for i in 0..<120 { append(&w.vehicles, Vehicle{road = 0, distance = f32(i)*8, kind = .Car}) }
    w.requested, w.spawned = 120, 120
    step(&w, 0.05)
    testing.expect(t, len(w.vehicles) == 120 && w.pending == 1 && w.requested == w.spawned+w.pending)
    ordered_remove(&w.vehicles, 0)
    w.completed += 1
    step(&w, 0.05)
    testing.expect(t, len(w.vehicles) == 120 && w.pending == 1 && w.requested == w.spawned+w.pending)
    testing.expect(t, w.spawned == len(w.vehicles)+w.completed)
}
