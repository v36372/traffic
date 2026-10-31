package sim

import "core:testing"

passing_world :: proc(w: ^World, blocked: bool) {
    init(w, []Node{{{0, 0}}, {{300, 0}}}, []Road{{0, 1, 8, 9}}, 21)
    config := default_driver_config()
    config.enabled = true
    config.classes[int(Vehicle_Kind.Motorcycle)].speed_min = 0.5
    assert(configure_drivers(w, config))
    w.spawn_accumulator = -1000
    append(&w.vehicles, Vehicle{road = 0, distance = 55, speed = 4.5, kind = .Bicycle, spawn_index = -1, driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}})
    append(&w.vehicles, Vehicle{road = 0, distance = 30, speed = 8, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}, side = -1}})
    if blocked {
        w.vehicles[0].distance, w.vehicles[0].speed, w.vehicles[0].driver.traits.speed_factor = 34, 0, 0
        append(&w.vehicles, Vehicle{road = 0, distance = 34, lateral = -1.1, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 0, headway = 0.8, acceleration = 2}}})
        append(&w.vehicles, Vehicle{road = 0, distance = 34, lateral = 1.1, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 0, headway = 0.8, acceleration = 2}}})
    }
}

@(test)
test_overtaking_and_replay :: proc(t: ^testing.T) {
    a, b: World
    passing_world(&a, false); passing_world(&b, false)
    defer destroy(&a); defer destroy(&b)
    turned := false
    for _ in 0..<300 {
        step(&a, 0.05); step(&b, 0.05)
        testing.expect(t, len(a.vehicles) == len(b.vehicles))
        for v, i in a.vehicles {
            testing.expect(t, v == b.vehicles[i])
            _, f := vehicle_pose(&a, v)
            extent := abs(f.y)*kind_length(v.kind)*0.5+abs(f.x)*kind_width(v.kind)*0.5
            testing.expect(t, abs(v.lateral)+extent <= a.roads[v.road].width*0.5)
            for other in a.vehicles[i+1:] { testing.expect(t, !bodies_overlap(&a, v, other)) }
        }
        _, f := vehicle_pose(&a, a.vehicles[1])
        if abs(f.y) > 0.001 { turned = true }
    }
    testing.expect(t, turned && a.vehicles[1].distance > a.vehicles[0].distance)
}

@(test)
test_overtaking_blocked :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, true)
    defer destroy(&w)
    for _ in 0..<100 {
        step(&w, 0.05)
        testing.expect(t, !w.vehicles[1].lane.active && w.vehicles[1].lateral == 0)
        for v, i in w.vehicles { for other in w.vehicles[i+1:] { testing.expect(t, !bodies_overlap(&w, v, other)) } }
    }
    testing.expect(t, w.vehicles[1].distance < 34-1.8)
}

@(test)
test_car_pass_and_narrow_road :: proc(t: ^testing.T) {
    for width in ([]f32{3, 10}) {
        w: World
        passing_world(&w, false)
        w.roads[0].width = width
        w.behavior.classes[int(Vehicle_Kind.Car)].speed_min = 0.5
        w.vehicles[1].kind = .Car
        for _ in 0..<300 {
            step(&w, 0.05)
            testing.expect(t, !bodies_overlap(&w, w.vehicles[0], w.vehicles[1]))
            if width == 3 { testing.expect(t, !w.vehicles[1].lane.active && w.vehicles[1].lateral == 0) }
        }
        if width == 10 { testing.expect(t, w.vehicles[1].distance > w.vehicles[0].distance) }
        destroy(&w)
    }
}

@(test)
test_no_lane_change_in_connector_or_signal_approach :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    w.vehicles[1].motion.active = true
    try_overtake(&w, 1)
    testing.expect(t, !w.vehicles[1].lane.active)
    w.vehicles[1].motion = {}
    append(&w.signals, Signal_Plan{node = 1, green = 1, amber = 1, clearance = 100, offset = 2})
    cache_geometry(&w)
    stop := entry_distance(&w, 0, .Motorcycle)
    w.vehicles[1].distance, w.vehicles[1].speed = stop-5, 0
    w.vehicles[0].distance, w.vehicles[0].speed, w.vehicles[0].driver.traits.speed_factor = stop-1, 0, 0
    for _ in 0..<100 {
        step(&w, 0.05)
        v := w.vehicles[1]
        _, f := vehicle_pose(&w, v)
        testing.expect(t, signal_state(&w, 0) == .Red && !v.lane.active && !v.motion.active && v.distance <= stop && f.y == 0)
        testing.expect(t, !bodies_overlap(&w, v, w.vehicles[0]))
    }
}

@(test)
test_overtake_excludes_junctions_and_receiving_reservations :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    cache_geometry(&w)
    w.vehicles[1].distance = 5
    w.vehicles[0].distance = 30
    try_overtake(&w, 1)
    testing.expect(t, !w.vehicles[1].lane.active)
    w.vehicles[1].distance = entry_distance(&w, 0, .Motorcycle)-10
    w.vehicles[0].distance = w.vehicles[1].distance+25
    try_overtake(&w, 1)
    testing.expect(t, !w.vehicles[1].lane.active)
    w.vehicles[1].distance, w.vehicles[0].distance = 30, 55
    w.vehicles[0].motion = Connector{active = true, exit = 0}
    try_overtake(&w, 1)
    testing.expect(t, !w.vehicles[1].lane.active)
    w.vehicles[0].motion = {}
    try_overtake(&w, 1)
    testing.expect(t, w.vehicles[1].lane.active)
}

@(test)
test_seeded_side_and_rear_approach :: proc(t: ^testing.T) {
    left, right := false, false
    for id in u64(1)..=u64(20) {
        w: World
        passing_world(&w, false)
        cache_geometry(&w)
        w.vehicles[1].id = id
        w.vehicles[1].driver.side = -1
        if driver_sample(w.behavior_seed, id, 0x082efa98ec4e6c89) >= 0.5 { w.vehicles[1].driver.side = 1 }
        original_rng := w.rng
        try_overtake(&w, 1)
        testing.expect(t, w.vehicles[1].lane.active && w.rng == original_rng)
        if w.vehicles[1].lane.to < 0 { left = true } else { right = true }
        destroy(&w)
    }
    testing.expect(t, left && right)

    follower: World
    passing_world(&follower, false)
    defer destroy(&follower)
    cache_geometry(&follower)
    follower.vehicles[1].speed = 3
    append(&follower.vehicles, Vehicle{road = 0, distance = 20, speed = 9, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}})
    try_overtake(&follower, 1)
    testing.expect(t, follower.vehicles[1].lane.active)
    for _ in 0..<200 {
        step(&follower, 0.05)
        for v, i in follower.vehicles { for other in follower.vehicles[i+1:] { testing.expect(t, !bodies_overlap(&follower, v, other)) } }
    }
    testing.expect(t, !follower.vehicles[1].lane.active && abs(follower.vehicles[1].lateral) > 0.5)

    blocked: World
    passing_world(&blocked, false)
    defer destroy(&blocked)
    cache_geometry(&blocked)
    blocked.vehicles[1].speed = 3
    for side in ([]f32{-1, 1}) {
        append(&blocked.vehicles, Vehicle{road = 0, distance = 20, lateral = side, speed = 9, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}})
    }
    try_overtake(&blocked, 1)
    testing.expect(t, !blocked.vehicles[1].lane.active)

    w: World
    passing_world(&w, false)
    defer destroy(&w)
    cache_geometry(&w)
    w.vehicles[1].speed = 3
    try_overtake(&w, 1)
    testing.expect(t, w.vehicles[1].lane.active)
    side := w.vehicles[1].lane.to
    append(&w.vehicles, Vehicle{road = 0, distance = 20, lateral = side, speed = 9, kind = .Motorcycle, spawn_index = -1, driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}})
    distance := w.vehicles[1].distance
    for _ in 0..<10 {
        step(&w, 0.05)
        testing.expect(t, w.vehicles[1].distance == distance && w.vehicles[1].lateral == 0)
        for v, i in w.vehicles { for other in w.vehicles[i+1:] { testing.expect(t, !bodies_overlap(&w, v, other)) } }
    }
    w.vehicles[2].speed, w.vehicles[2].driver.traits.speed_factor = 0, 0
    for _ in 0..<200 {
        step(&w, 0.05)
        for v, i in w.vehicles { for other in w.vehicles[i+1:] { testing.expect(t, !bodies_overlap(&w, v, other)) } }
    }
    testing.expect(t, !w.vehicles[1].lane.active && abs(w.vehicles[1].lateral) > 0.5)
}

@(test)
test_motorcycle_pass_on_showcase_width :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    w.roads[0].width = 3.5
    for _ in 0..<300 {
        step(&w, 0.05)
        v := w.vehicles[1]
        _, f := vehicle_pose(&w, v)
        extent := abs(f.y)*kind_length(v.kind)*0.5+abs(f.x)*kind_width(v.kind)*0.5
        testing.expect(t, abs(v.lateral)+extent <= 1.75 && !bodies_overlap(&w, v, w.vehicles[0]))
    }
    testing.expect(t, w.vehicles[1].distance > w.vehicles[0].distance)
}

@(test)
test_fixed_speed_car_can_pass_in_varied_scenario :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    w.roads[0].width = 10
    w.vehicles[0].kind, w.vehicles[0].speed, w.vehicles[0].distance = .Bus, 5.5, 64
    w.vehicles[1].kind, w.vehicles[1].speed = .Car, 7
    turned := false
    for _ in 0..<600 {
        step(&w, 0.05)
        testing.expect(t, !bodies_overlap(&w, w.vehicles[0], w.vehicles[1]))
        if w.vehicles[1].lane.active { turned = true }
    }
    testing.expect(t, turned && w.vehicles[1].distance > w.vehicles[0].distance)
}

@(test)
test_near_end_yielding_rear_recovers :: proc(t: ^testing.T) {
    for reversed in ([]bool{false, true}) {
        w: World
        passing_world(&w, false)
        defer destroy(&w)
        clear(&w.vehicles)
        passer := Vehicle{id = 41, road = 0, distance = 68.079, lateral = -1.395, kind = .Motorcycle, spawn_index = -1,
            lane = {active = true, start = 56.579, from = -0.345, to = -1.395},
            driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}}
        rear := Vehicle{id = 47, road = 0, distance = 66.129, lateral = -1.395, kind = .Motorcycle, spawn_index = -1,
            driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 2}}}
        pi, ri := 0, 1
        if reversed { append(&w.vehicles, rear, passer); pi, ri = 1, 0 } else { append(&w.vehicles, passer, rear) }
        cache_geometry(&w)
        testing.expect(t, !bodies_overlap(&w, passer, rear))
        testing.expect(t, lane_rear_safe(&w, pi, passer, false))
        testing.expect(t, !lane_rear_safe(&w, pi, passer, true))
        w.vehicles[ri].speed = 1
        testing.expect(t, !lane_rear_safe(&w, pi, passer, false))
        blocked, _ := move_vehicle(&w, pi, 0.1)
        testing.expect(t, blocked == 0 && w.vehicles[pi].distance == passer.distance)
        w.vehicles[ri].speed = 0
        // The yielding exception must not bypass the actual body check.
        obstacle := rear
        obstacle.id, obstacle.distance = 48, passer.distance+kind_length(.Motorcycle)+0.11
        append(&w.vehicles, obstacle)
        blocked, _ = move_vehicle(&w, pi, 0.1)
        testing.expect(t, blocked == 0 && !bodies_overlap(&w, w.vehicles[pi], obstacle))
        pop(&w.vehicles)
        for _ in 0..<100 {
            step(&w, 0.05)
            testing.expect(t, !bodies_overlap(&w, w.vehicles[pi], w.vehicles[ri]))
        }
        testing.expect(t, !w.vehicles[pi].lane.active && w.vehicles[pi].distance > 68.579)
    }
}

@(test)
test_impatient_motorcycle_takes_small_gap_after_waiting :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    cache_geometry(&w)
    w.vehicles[0].speed = 8.2
    w.vehicles[1].driver = {
        persona = .Cautious, kind = .Motorcycle, side = -1, aggressiveness = 0.2, patience = 0.5,
        traits = {speed_factor = 1, headway = 0.8, acceleration = 2},
    }
    try_overtake(&w, 1)
    testing.expect(t, !w.vehicles[1].lane.active)
    w.vehicles[1].driver.waited = 0.5
    try_overtake(&w, 1)
    testing.expect(t, w.vehicles[1].lane.active && w.vehicles[1].lane.length == 6)
    testing.expect(t, w.vehicles[1].driver.mode == .Filtering)
}

@(test)
test_assertive_motorcycle_may_cut_occupied_band :: proc(t: ^testing.T) {
    testing.expect(t, motorcycle_may_cut(Vehicle{kind = .Motorcycle, driver = {aggressiveness = 0.9}}))
    testing.expect(t, !motorcycle_may_cut(Vehicle{kind = .Motorcycle, driver = {aggressiveness = 0.2}}))
    testing.expect(t, !motorcycle_may_cut(Vehicle{kind = .Car, driver = {aggressiveness = 0.9}}))
}
