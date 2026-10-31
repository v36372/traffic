package sim

import "core:math"
import "core:testing"

@(test)
test_signal_phases_and_offset :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{10,0}}, {{10,10}}, {{20,0}}}, []Road{{0,1,4,8}, {2,1,4,8}, {1,3,4,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1, 2, 1, 1, 1, false}}, nil))
    testing.expect(t, signal_state(&w, 0) == .Green && signal_state(&w, 1) == .Red)
    w.time = 1; testing.expect(t, signal_state(&w, 0) == .Amber)
    w.time = 2; testing.expect(t, signal_state(&w, 0) == .Red && signal_state(&w, 1) == .Red)
    w.time = 3; testing.expect(t, signal_state(&w, 1) == .Green && signal_state(&w, 0) == .Red)
    w.time = 7; testing.expect(t, signal_state(&w, 0) == .Green)
    testing.expect(t, signal_state(&w, 2) == .Uncontrolled)
}

@(test)
test_make_signal_timings_countdown_and_remaining :: proc(t: ^testing.T) {
    spec := default_signal_spec()
    testing.expect(t, spec.green == 8 && spec.amber == 3 && spec.clearance == 2 && spec.offset == 0 && !spec.countdown)
    spec.node = 1
    spec.green, spec.amber, spec.clearance, spec.offset = 4, 2, 1, 0
    spec.countdown = true
    plan, ok := make_signal(spec)
    testing.expect(t, ok && plan.countdown && plan.green == 4 && plan.amber == 2 && plan.clearance == 1)
    bad := spec
    bad.green = 0
    _, bad_ok := make_signal(bad)
    testing.expect(t, !bad_ok)
    bad = spec
    bad.node = -1
    _, bad_ok = make_signal(bad)
    testing.expect(t, !bad_ok)
    w: World
    init(&w, []Node{{{0,0}}, {{40,0}}, {{40,40}}, {{80,0}}}, []Road{{0,1,4,8}, {2,1,4,8}, {1,3,4,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{plan}, nil) && w.signals[0].countdown)
    state, remaining, got, found := signal_phase(&w, 0)
    testing.expect(t, found && got.countdown && state == .Green && math.abs(remaining-4) < 0.001)
    testing.expect(t, signal_countdown_seconds(remaining) == 4)
    w.time = 3.2
    state, remaining, _, _ = signal_phase(&w, 0)
    testing.expect(t, state == .Green && remaining > 0.79 && remaining < 0.81)
    testing.expect(t, signal_countdown_seconds(remaining) == 1)
    w.time = 4
    state, remaining, _, _ = signal_phase(&w, 0)
    testing.expect(t, state == .Amber && math.abs(remaining-2) < 0.001)
    w.time = 7
    state, remaining, _, _ = signal_phase(&w, 0)
    other, other_left, _, _ := signal_phase(&w, 1)
    testing.expect(t, state == .Red && other == .Green)
    testing.expect(t, remaining > other_left && remaining <= 4+2+1+4)
    testing.expect(t, signal_countdown_seconds(0) == 0 && signal_countdown_seconds(-1) == 0)
}

@(test)
test_signal_stop_release_and_committed :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{30,0}}, {{30,30}}, {{60,0}}}, []Road{{0,1,4,8}, {2,1,4,8}, {1,3,4,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1, 2, 1, 1, 2, false}}, []Spawn_Point{{0,3,86400,.Car}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Car)-6, speed = 8, kind = .Car})
    // Amber, then red: front must remain behind the line.
    for _ in 0..<35 { step(&w, 0.05) }
    testing.expect(t, w.vehicles[0].road == 0 && w.vehicles[0].distance + kind_length(.Car)*0.5 <= stop_line_distance(&w, 0) + 0.001)
    // After the other approach's slot, green releases the queue.
    for _ in 0..<260 { step(&w, 0.05) }
    testing.expect(t, w.completed == 1 || w.vehicles[0].road == 2)

    clear(&w.vehicles)
    w.time = 2 // other approach green, first red
    append(&w.vehicles, Vehicle{road = 0, distance = 28, speed = 1, kind = .Car})
    for _ in 0..<30 { step(&w, 0.05) }
    testing.expect(t, len(w.vehicles) == 1 && (w.vehicles[0].motion.active || w.vehicles[0].road == 2))
    for _ in 0..<100 { step(&w, 0.05) }
    testing.expect(t, w.completed > 0 || w.vehicles[0].road == 2)
}

@(test)
test_routes_rollback_and_demand :: proc(t: ^testing.T) {
    nodes := []Node{{{0,0}}, {{40,0}}, {{80,0}}, {{120,0}}, {{80,40}}}
    roads := []Road{{0,1,3,10}, {1,2,3,10}, {2,1,3,10}, {2,3,3,10}, {1,4,1,10}, {1,0,3,10}}
    a, b: World
    init(&a, nodes, roads, 42); init(&b, nodes, roads, 42)
    defer destroy(&a); defer destroy(&b)
    spawn := []Spawn_Point{{0,3,0.05,.Car}}
    testing.expect(t, configure(&a, nil, spawn) && configure(&b, nil, spawn))
    spawn[0].destination = 4
    testing.expect(t, !configure(&a, nil, spawn))
    testing.expect(t, a.spawns[0].destination == 3)
    testing.expect(t, !configure(&a, nil, []Spawn_Point{{0,0,1,.Car}}))
    testing.expect(t, !configure(&a, []Signal_Plan{{1,1,1,1,0, false}, {1,1,1,1,0, false}}, nil))
    for _ in 0..<4000 {
        step(&a, 0.05); step(&b, 0.05)
        testing.expect(t, a.requested == a.spawned + a.pending)
        testing.expect(t, a.spawned == a.completed + len(a.vehicles))
        testing.expect(t, len(a.vehicles) <= 120)
        testing.expect(t, a.requested == b.requested && a.pending == b.pending && a.completed == b.completed && len(a.vehicles) == len(b.vehicles))
        for v, i in a.vehicles { testing.expect(t, v == b.vehicles[i] && v.road != 2 && v.road != 4) }
    }
    testing.expect(t, a.pending > 0 && a.completed > 0)
}

@(test)
test_short_controlled_approach_rejected :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{2,0}}, {{2,10}}, {{20,0}}}, []Road{{0,1,3,8}, {2,1,3,8}, {1,3,3,8}})
    defer destroy(&w)
    testing.expect(t, !configure(&w, []Signal_Plan{{1,2,1,1,0, false}}, []Spawn_Point{{0,3,0.05,.Bus}}))
}

@(test)
test_submillimetre_controlled_approach_rejected :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{0.0005,0}}, {{0,10}}, {{20,0}}}, []Road{{0,1,3,8}, {2,1,3,8}, {1,3,3,8}})
    defer destroy(&w)
    testing.expect(t, !configure(&w, []Signal_Plan{{1,2,1,1,0, false}}, []Spawn_Point{{0,3,86400,.Car}}))
}

@(test)
test_short_long_vehicle_source_pending_is_explicit :: proc(t: ^testing.T) {
    // The 4m crossing fits, but this 11m approach cannot queue a 10m bus
    // behind its upstream bar; keep pending/diagnostic rather than spawning.
    nodes := []Node{{{0,0}}, {{11,0}}, {{11,20}}, {{41,0}}}
    roads := []Road{{0,1,3,8}, {2,1,3,8}, {1,3,3,8}}
    plan := []Signal_Plan{{1,2,1,1,0, false}}
    w: World
    init(&w, nodes, roads, 34)
    defer destroy(&w)
    testing.expect(t, configure(&w, plan, []Spawn_Point{{0,3,0.05,.Bus}}))
    testing.expect(t, stop_line_distance(&w, 0) > 0 && stop_line_distance(&w, 0) < kind_length(.Bus)*0.5)
    w.time = 4
    step(&w, 0.05)
    testing.expect(t, len(w.vehicles) == 0 && w.pending == 1)
    _, fits := fitting_lateral(&w, 0, .Bus, 0)
    testing.expect(t, !fits && !free_lateral(&w, 0, 0, .Bus, 0))
    w.time = 8
    step(&w, 0.05)
    testing.expect(t, len(w.vehicles) == 0 && w.pending == 2)
    for _ in 0..<400 { step(&w, 0.05) }
    testing.expect(t, w.completed == 0 && w.pending == w.requested)
    testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Short_Approach)

    // The synthetic source also goes through free_lateral, with a seeded bus draw.
    synthetic: World
    init(&synthetic, nodes, roads, 34)
    defer destroy(&synthetic)
    testing.expect(t, configure(&synthetic, plan, nil))
    synthetic.time = 4
    try_spawn(&synthetic)
    testing.expect(t, len(synthetic.vehicles) == 0)
    synthetic.rng = 34
    synthetic.time = 8
    try_spawn(&synthetic)
    testing.expect(t, len(synthetic.vehicles) == 0 && len(synthetic.motion_issues) == 1)
}

@(test)
test_short_source_policy_matches_terminal_and_continuing_routes :: proc(t: ^testing.T) {
    destinations := []int{1,3}
    for destination in destinations {
        w: World
        init(&w, []Node{{{0,0}}, {{11,0}}, {{11,20}}, {{41,0}}}, []Road{{0,1,3,8}, {2,1,3,8}, {1,3,3,8}})
        testing.expect(t, configure(&w, []Signal_Plan{{1,2,1,1,0, false}}, []Spawn_Point{{0,destination,0.05,.Bus}}))
        exit := destination == 1 ? -1 : 2
        testing.expect(t, entry_distance(&w, 0, .Bus) < 0 && successor(&w, Vehicle{road = 0, kind = .Bus}) == exit)
        step(&w, 0.05)
        testing.expect(t, signal_state(&w, 0) == .Green)
        testing.expect(t, w.spawned == 0 && len(w.vehicles) == 0 && w.pending == 1 && w.requested == 1,
            "terminal and continuing trips must share short-approach source validation on green")
        testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0] == Motion_Issue{0,exit,.Short_Approach})
        w.spawns[0].interval = 86400
        w.spawn_runtime[0].elapsed = 0
        w.time = 3.1
        step(&w, 0.5)
        testing.expect(t, signal_state(&w, 0) == .Red && w.spawned == 0 && len(w.vehicles) == 0 && w.pending == 1 && w.requested == 1 && len(w.motion_issues) == 1)
        destroy(&w)
    }
}

@(test)
test_long_terminal_source_red_queue_and_green_completion :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{60,0}}, {{60,60}}, {{120,0}}}, []Road{{0,1,6,8}, {2,1,6,8}, {1,3,6,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0, false}}, []Spawn_Point{{0,1,0.05,.Bus}}))
    step(&w, 0.05)
    testing.expect(t, w.spawned == 1 && len(w.vehicles) == 1 && w.pending == 0 && len(w.motion_issues) == 0)
    w.spawns[0].interval = 86400
    w.spawn_runtime[0].elapsed = 0
    w.signals[0].offset, w.time = 102, 0 // opposite approach owns a long green
    line := entry_distance(&w, 0, .Bus)
    for _ in 0..<300 {
        step(&w, 0.05)
        testing.expect(t, signal_state(&w, 0) == .Red && len(w.vehicles) == 1)
        testing.expect(t, w.vehicles[0].distance <= line+0.001 && !w.vehicles[0].motion.active)
    }
    testing.expect(t, abs(w.vehicles[0].distance-line) < 0.01 && w.vehicles[0].speed == 0)
    w.signals[0].offset, w.time = 0, 0
    step(&w, 0.05)
    testing.expect(t, w.vehicles[0].motion.active && w.vehicles[0].motion.node == 1 && w.vehicles[0].motion.exit == -1,
        "terminal trips must reserve at the bar before crossing the band too")
    for _ in 0..<400 { step(&w, 0.05) }
    testing.expect(t, len(w.vehicles) == 0 && w.completed == 1 && w.spawned == 1 && w.requested == 1 && w.pending == 0)
}
