package sim

import "core:testing"

@(test)
test_connector_speed_targets_and_clearance :: proc(t: ^testing.T) {
    for mode in 0..<4 {
        w: World
        exit_node := mode == 1 ? Vec2{0,60} : Vec2{60,0}
        exit_limit: f32 = mode == 2 ? 2.5 : 12
        incoming_limit: f32 = mode == 3 ? 12 : 8
        kind := mode == 3 ? Vehicle_Kind.Car : Vehicle_Kind.Motorcycle
        init(&w, []Node{{{-60,0}}, {{0,0}}, {exit_node}}, []Road{{0,1,3.5,incoming_limit}, {1,2,3.5,exit_limit}}, 42)
        defer destroy(&w)
        testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,kind}}))
        append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, kind), kind = kind, id = 1})
        testing.expect(t, acquire_connector(&w, 0, 1))
        _, straight := straight_direction(w.vehicles[0].motion)
        testing.expect(t, straight == (mode != 1))
        max_speed: f32
        finished := false
        for _ in 0..<800 {
            step(&w, 0.05)
            if len(w.vehicles) == 0 { finished = true; break }
            v := w.vehicles[0]
            if v.motion.active && v.road == 0 {
                max_speed = max(max_speed, v.speed)
                testing.expect(t, v.motion.progress < v.motion.length)
            }
        }
        testing.expect(t, finished)
        if mode == 0 { testing.expect(t, max_speed > 4.5 && max_speed <= min(8, kind_speed(.Motorcycle))+0.001) }
        if mode == 1 { testing.expect(t, max_speed > 3.9 && max_speed <= 4.001) }
        if mode == 2 { testing.expect(t, max_speed > 2.4 && max_speed <= 2.501) }
        if mode == 3 { testing.expect(t, max_speed > 6.5 && max_speed <= kind_speed(.Car)+0.001) }
    }
}

@(test)
test_brakes_for_slower_exit_before_commitment :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-100,0}}, {{0,0}}, {{100,0}}}, []Road{{0,1,3.5,8}, {1,2,3.5,2.5}}, 42)
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    stop := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles, Vehicle{road = 0, distance = stop-25, speed = 8, kind = .Motorcycle, id = 1})
    committed := false
    for _ in 0..<300 {
        step(&w, 0.05)
        if w.vehicles[0].motion.active {
            committed = true
            // Tick integration permits a small braking discretization excess.
            testing.expect(t, w.vehicles[0].speed <= 3.0)
            break
        }
    }
    testing.expect(t, committed)
}

@(test)
test_curve_entry_decelerates_without_instant_speed_clamp :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{0,60}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}}, 42)
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), speed = 8, kind = .Motorcycle})
    testing.expect(t, acquire_connector(&w, 0, 1))
    _, straight := straight_direction(w.vehicles[0].motion)
    testing.expect(t, !straight)
    step(&w, 0.05)
    testing.expect(t, abs(w.vehicles[0].speed-7.8) < 0.001, "4 m/s is a target; existing finite braking does not instantly clamp entry speed")
}

@(test)
test_turn_approach_brakes_before_commitment :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-100,0}}, {{0,0}}, {{0,100}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}}, 42)
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    stop := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles, Vehicle{road = 0, distance = stop-25, speed = 8, kind = .Motorcycle, id = 1})
    committed, released := false, false
    for _ in 0..<500 {
        step(&w, 0.05)
        if len(w.vehicles) == 0 { break }
        v := w.vehicles[0]
        if v.motion.active && v.road == 0 && !committed {
            committed = true
            _, straight := straight_direction(v.motion)
            testing.expect(t, !straight && v.speed <= 4.3, "approaching 90-degree turn must brake near 4 m/s before commitment")
            hull := junction_polygon(&w, v.motion.node)
            testing.expect(t, connector_on_pavement(&w, v.motion, v.kind, hull[:]))
            delete(hull)
        }
        if committed && v.road == 1 { released = true; break }
    }
    testing.expect(t, committed && released)
}

@(test)
test_reversing_approach_brakes_even_if_transition_is_unsupported :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-100,0}}, {{0,0}}}, []Road{{0,1,3.5,8}, {1,0,3.5,8}}, 42)
    defer destroy(&w)
    w.spawn_accumulator = -100
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle)-2, speed = 8, kind = .Motorcycle})
    step(&w, 0.05)
    testing.expect(t, abs(w.vehicles[0].speed-7.8) < 0.001 && !w.vehicles[0].motion.active)
}

@(test)
test_parallel_lateral_shift_retains_curve_target :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{40,0}}, {{120,0}}}, []Road{{0,1,6,8}, {1,2,2,8}}, 42)
    defer destroy(&w)
    w.spawn_accumulator = -100
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Car), lateral = 1.5, preferred_lateral = 1.5, speed = 7, kind = .Car, spawn_index = -1})
    testing.expect(t, acquire_connector(&w, 0, 1))
    _, straight := straight_direction(w.vehicles[0].motion)
    testing.expect(t, !straight, "aligned road headings do not prove a narrowing connector is straight")
    step(&w, 0.05)
    testing.expect(t, abs(w.vehicles[0].speed-6.8) < 0.001)
}
