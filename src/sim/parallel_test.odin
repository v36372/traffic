package sim

import "core:testing"

@(test)
test_disjoint_straight_motorcycles_share_node_and_drain_deterministically :: proc(t: ^testing.T) {
    a, b: World
    worlds := [2]^World{&a, &b}
    for w in worlds {
        init(w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}}, 42)
        testing.expect(t, configure(w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
        for row in 0..<2 {
            for band in 0..<3 {
                lateral := source_lateral(3.5, .Motorcycle, band)
                append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(w, 0, .Motorcycle)-f32(row)*3.5,
                    lateral = lateral, preferred_lateral = lateral, speed = 2, kind = .Motorcycle, id = u64(row*3+band+1)})
            }
        }
        w.spawned, w.requested = 6, 6
    }
    defer destroy(&a)
    defer destroy(&b)
    max_owners := 0
    for _ in 0..<1400 {
        step(&a, 0.05)
        step(&b, 0.05)
        owners := 0
        for v, i in a.vehicles {
            testing.expect(t, v == b.vehicles[i])
            if v.motion.active { owners += 1 }
        }
        max_owners = max(max_owners, owners)
        expect_bodies_clear(t, &a)
        testing.expect(t, a.requested == a.spawned+a.pending && a.spawned == a.completed+len(a.vehicles))
        testing.expect(t, a.spawned == b.spawned && a.completed == b.completed && len(a.vehicles) == len(b.vehicles))
    }
    testing.expect(t, max_owners > 3, "the following row must enter before the whole leading row finishes its paths")
    testing.expect(t, a.completed == 6 && len(a.vehicles) == 0 && a.pending == 0 && len(a.motion_issues) == 0)
    testing.expect(t, a.rng == 42 && b.rng == 42)
}

@(test)
test_parallel_car_and_bike_share_but_conflicting_turn_waits :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}, {3,1,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Car}, {2,2,86400,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Car), lateral = -0.6, preferred_lateral = -0.6, speed = 2, kind = .Car, id = 1})
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), lateral = 0.93, preferred_lateral = 0.93, speed = 2, kind = .Motorcycle, id = 2})
    append(&w.vehicles, Vehicle{road = 2, distance = entry_distance(&w, 2, .Motorcycle), speed = 2, kind = .Motorcycle, spawn_index = 1, id = 3})
    w.spawned, w.requested = 3, 3
    step(&w, 0.05)
    saw_turn := false
    for _ in 0..<1400 {
        step(&w, 0.05)
        for v in w.vehicles {
            if v.id == 3 && v.motion.active { saw_turn = true }
        }
        expect_bodies_clear(t, &w)
    }
    testing.expect(t, saw_turn && w.completed == 3 && len(w.vehicles) == 0)
}

@(test)
test_distant_receiving_occupant_does_not_preoccupy_route :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    append(&w.vehicles,
        Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), kind = .Motorcycle, id = 1},
        Vehicle{road = 1, distance = 30, speed = 0, kind = .Motorcycle, id = 2})
    testing.expect(t, acquire_connector(&w, 0, 1), "a distant exit occupant is outside the rolling conflict window")
    testing.expect(t, w.vehicles[0].motion.active && !bodies_overlap(&w, w.vehicles[0], w.vehicles[1]))
    _, _ = move_vehicle(&w, 0, 0.5)
    expect_bodies_clear(t, &w)
}

@(test)
test_skew_and_converging_corridors_do_not_qualify_for_straight_batch :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{60,60}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}, {1,3,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    a := Vehicle{road = 0, lateral = -0.93, kind = .Motorcycle}
    straight, problem := make_connector(&w, a, 1, -0.93)
    testing.expect(t, problem == .None)
    converging, _ := make_connector(&w, a, 1, 0.93)
    skew, _ := make_connector(&w, a, 2, 0.93)
    testing.expect(t, !straight_corridors_disjoint(straight, .Motorcycle, converging, .Motorcycle))
    testing.expect(t, !straight_corridors_disjoint(straight, .Motorcycle, skew, .Motorcycle))
    almost := straight
    almost.controls[1].y += 0.00001
    _, accepted := straight_direction(almost)
    testing.expect(t, !accepted, "almost-straight sampled curves must not qualify for the analytic straight exception")
    close := straight
    close.start.y += 0.92
    for &point in close.controls { point.y += 0.92 }
    testing.expect(t, !straight_corridors_disjoint(straight, .Motorcycle, close, .Motorcycle), "less than the 0.18m body gap must serialize")
}

@(test)
test_parallel_commitment_clears_but_red_band_must_wait :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}, {3,1,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0, false}}, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    for band in 0..<2 {
        lateral := source_lateral(3.5, .Motorcycle, band)
        append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), lateral = lateral,
            preferred_lateral = lateral, speed = 2, kind = .Motorcycle, id = u64(band+1)})
    }
    step(&w, 0.05)
    testing.expect(t, w.vehicles[0].motion.active && w.vehicles[1].motion.active)
    progress := w.vehicles[0].motion.progress
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), lateral = 0.93,
        preferred_lateral = 0.93, speed = 2, kind = .Motorcycle, id = 3})
    w.spawned, w.requested = 3, 3
    w.signals[0].offset, w.time = 102, 0
    step(&w, 0.05)
    testing.expect(t, w.vehicles[0].motion.progress > progress && !w.vehicles[2].motion.active)
    for _ in 0..<200 {
        step(&w, 0.05)
        for v in w.vehicles {
            if v.id == 3 { testing.expect(t, v.road == 0 && !v.motion.active && v.distance <= entry_distance(&w, 0, v.kind)+0.001) }
        }
        expect_bodies_clear(t, &w)
    }
    w.signals[0].offset, w.time = 0, 0
    for _ in 0..<800 { step(&w, 0.05); expect_bodies_clear(t, &w) }
    testing.expect(t, w.completed == 3 && len(w.vehicles) == 0 && w.pending == 0 && len(w.motion_issues) == 0)
}

@(test)
test_aligned_receiving_band_accounts_for_unequal_reciprocal_offsets :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}}, []Road{{0,1,5.5,8}, {1,0,5.5,8}, {1,2,3.5,8}, {2,1,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    v := Vehicle{road = 0, lateral = 0, preferred_lateral = 0, kind = .Motorcycle}
    candidates := connector_laterals(&w, v, 2)
    defer delete(candidates)
    testing.expect(t, abs(candidates[0]+1) < 0.00001, "world corridor must account for both directed road offsets")
    c, problem := make_connector(&w, v, 2, candidates[0])
    _, straight := straight_direction(c)
    testing.expect(t, problem == .None && straight)
    v.lateral = -2
    outside := connector_laterals(&w, v, 2)
    defer delete(outside)
    testing.expect(t, outside[0] == 0, "an infeasible world corridor must not be clamped into a false straight")
}
