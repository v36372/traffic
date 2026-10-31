package sim

import "core:testing"

@(test)
test_local_gap_allows_parallel_movement_without_route_ownership :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}}, []Road{
        {0,1,5,8}, {1,2,5,8}, {1,0,5,8}, {1,3,5,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    entry := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles,
        Vehicle{road = 0, distance = entry, lateral = -1.2, kind = .Motorcycle, id = 1},
        Vehicle{road = 0, distance = entry, lateral = 1.2, kind = .Motorcycle, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 1))
    testing.expect(t, acquire_connector(&w, 1, 1), "the second safe local movement need not wait for route ownership")
    testing.expect(t, w.vehicles[0].motion.active && w.vehicles[1].motion.active)
    for _ in 0..<1200 {
        _, _ = move_vehicle(&w, 0, 0.1)
        _, _ = move_vehicle(&w, 1, 0.1)
        expect_bodies_clear(t, &w)
        if w.vehicles[0].road == 1 && w.vehicles[1].road == 1 { break }
    }
    testing.expect(t, w.vehicles[0].road == 1 && w.vehicles[1].road == 1)
}

@(test)
test_stopped_conflicting_leader_releases_local_gap :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}}, []Road{
        {0,1,4,8}, {1,2,4,8}, {3,1,4,8}, {1,4,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}, {2,4,86400,.Motorcycle}}))
    append(&w.vehicles,
        Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 8, kind = .Motorcycle, id = 1},
        Vehicle{road = 2, distance = entry_distance(&w, 2, .Motorcycle), speed = 8, kind = .Motorcycle, spawn_index = 1, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 1))
    w.vehicles[0].motion.progress = w.vehicles[0].motion.length*0.5
    w.vehicles[0].speed = 0
    testing.expect(t, !acquire_connector(&w, 1, 3), "a stopped body in the local crossing window must block")
    moved, _ := move_vehicle(&w, 0, 20)
    testing.expect(t, moved > 0 && w.vehicles[0].road == 1)
    testing.expect(t, acquire_connector(&w, 1, 3), "the conflicting approach proceeds after the stopped leader clears")
    for _ in 0..<1200 {
        _, _ = move_vehicle(&w, 0, 0.1)
        _, _ = move_vehicle(&w, 1, 0.1)
        expect_bodies_clear(t, &w)
        if w.vehicles[0].road == 1 && w.vehicles[1].road == 3 { break }
    }
    testing.expect(t, w.vehicles[0].road == 1 && w.vehicles[1].road == 3)
}

@(test)
test_distant_exit_occupant_and_short_next_link_remain_live :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{18,0}}, {{78,0}}}, []Road{
        {0,1,3.5,8}, {1,2,3.5,8}, {2,3,3.5,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}}))
    append(&w.vehicles,
        Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 2, kind = .Motorcycle, id = 1},
        Vehicle{road = 1, distance = 10, speed = 0, kind = .Motorcycle, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 1), "far receiving traffic must not reserve the whole exit route")
    clear(&w.vehicles)
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 2, kind = .Motorcycle, id = 1})
    saw_first, completed := false, false
    for _ in 0..<2400 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        for v in w.vehicles { if v.id == 1 && v.road == 1 { saw_first = true } }
        if w.completed == 1 { completed = true; break }
    }
    testing.expect(t, saw_first && completed && len(w.vehicles) == 0,
        "a short receiving link must not create a persistent queue-owner cycle")
}

@(test)
test_red_signal_still_prevents_local_gap_entry :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8}, {3,1,4,8}, {1,3,4,8}, {4,1,4,8}, {1,4,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0}}, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    w.time = 102
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 4, kind = .Motorcycle})
    for _ in 0..<100 {
        step(&w, 0.05)
        testing.expect(t, len(w.vehicles) == 1)
        testing.expect(t, signal_state(&w, 0) == .Red && !w.vehicles[0].motion.active && w.vehicles[0].distance <= entry_distance(&w, 0, .Motorcycle)+0.001)
        expect_bodies_clear(t, &w)
    }
}
