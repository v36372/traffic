package sim

import "core:testing"

@(test)
test_diagonal_left_turn_waits_for_opposing_traffic :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}, {{0,-60}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {2,1,4,8}, {1,2,4,8},
        {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}, {2,0,86400,.Car}}))
    entry := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles, Vehicle{road = 0, distance = entry, kind = .Motorcycle, id = 1},
        Vehicle{road = 2, distance = 57, kind = .Car, spawn_index = 1, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 4))
    testing.expect(t, w.vehicles[0].motion.lead < 0.001, "the turn starts at the entry, rather than at the middle of the intersection")
    moved, _ := move_vehicle(&w, 0, 30)
    _, f := vehicle_pose(&w, w.vehicles[0])
    testing.expect(t, moved > 0 && w.vehicles[0].motion.active && f.x > 0.1 && f.y > 0.1,
        "the driver enters diagonally and waits for the opposing body")
    expect_bodies_clear(t, &w)
    waiting := w.vehicles[0].motion.progress
    moved, _ = move_vehicle(&w, 0, 1)
    testing.expect(t, moved == 0 && w.vehicles[0].motion.progress == waiting)
    // Move the opposing vehicle away, then verify the turn can finish.
    w.vehicles[1].distance = 30
    moved, _ = move_vehicle(&w, 0, 30)
    testing.expect(t, moved > 0 && !w.vehicles[0].motion.active && w.vehicles[0].road == 4)
    expect_bodies_clear(t, &w)
}

@(test)
test_left_turn_completes_decisively_after_yielding :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}, {{0,-60}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {2,1,4,8}, {1,2,4,8},
        {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}, {2,0,86400,.Car}}))
    entry := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles, Vehicle{road = 0, distance = entry, kind = .Motorcycle, id = 1},
        Vehicle{road = 2, distance = 57, kind = .Car, spawn_index = 1, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 4))
    testing.expect(t, w.vehicles[0].motion.lead < 0.001 && connector_is_left(w.vehicles[0].motion))
    moved, _ := move_vehicle(&w, 0, 30)
    _, f := vehicle_pose(&w, w.vehicles[0])
    testing.expect(t, moved > 0 && w.vehicles[0].motion.active && f.x > 0.1 && f.y > 0.1)
    waiting := w.vehicles[0].motion.progress
    moved, _ = move_vehicle(&w, 0, 1)
    testing.expect(t, moved == 0 && w.vehicles[0].motion.progress == waiting)
    w.vehicles[0].speed = 4
    w.vehicles[0].driver.waited = 0.4
    w.vehicles[0].driver.mode = .Following_Junction
    w.vehicles[1].distance = 20
    w.vehicles[1].motion = {}
    punched := false
    max_speed: f32
    for _ in 0..<400 {
        step(&w, 0.05)
        if len(w.vehicles) == 0 { break }
        v := w.vehicles[0]
        if v.kind != .Motorcycle { continue }
        if v.motion.active && connector_is_left(v.motion) {
            max_speed = max(max_speed, v.speed)
            if v.speed > 4.5 { punched = true }
        }
        if !v.motion.active && v.road == 4 { break }
    }
    testing.expect(t, punched && max_speed > 4.5 && max_speed <= 6.51,
        "once the opposing body is gone the left finishes above the 4 m/s curve crawl")
}
