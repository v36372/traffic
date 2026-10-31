package sim

import "core:testing"

@(test)
test_local_gap_rejects_conflicting_forecast_but_not_distant_traffic :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}}, []Road{
        {0,1,4,8}, {1,2,4,8}, {3,1,4,8}, {1,4,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}, {2,4,86400,.Motorcycle}}))
    a := Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 4, kind = .Motorcycle}
    b := Vehicle{road = 2, distance = entry_distance(&w, 2, .Motorcycle), speed = 4, kind = .Motorcycle, spawn_index = 1}
    c, problem := make_connector(&w, a, 1, 0)
    testing.expect(t, problem == .None)
    a.motion = c
    b.motion, problem = make_connector(&w, b, 3, 0)
    testing.expect(t, problem == .None)
    append(&w.vehicles, a, b)
    testing.expect(t, !local_gap_clear(&w, a, 0, 4), "crossing trajectories inside the 1.5s window must conflict")

    w.vehicles[1] = Vehicle{road = 1, distance = 30, speed = 0, kind = .Motorcycle}
    testing.expect(t, local_gap_clear(&w, a, 0, 4), "a stopped vehicle outside the local conflict window is not a route reservation")
}

@(test)
test_forecast_stops_at_uncommitted_next_entry :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{120,0}}}, []Road{
        {0,1,4,8}, {1,2,4,8}, {2,3,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}}))
    v := Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 4, kind = .Motorcycle}
    c, problem := make_connector(&w, v, 1, 0)
    testing.expect(t, problem == .None)
    v.motion = c
    at_next_entry := forecast_vehicle(&w, v, c.length+100)
    testing.expect(t, !at_next_entry.motion.active && at_next_entry.road == 1)
    testing.expect(t, at_next_entry.distance <= entry_distance(&w, 1, .Motorcycle),
        "rolling prediction must stop at the next uncommitted entry, not own another route segment")
}
