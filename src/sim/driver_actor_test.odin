package sim

import "core:testing"

@(test)
test_actor_events_own_maneuvers_and_shared_recovery_transitions :: proc(t: ^testing.T) {
    d, ok := make_driver(default_driver_config(), 1, 1, .Motorcycle, .Balanced)
    testing.expect(t, ok)
    o := Driver_Observation{following = true, can_recover = true, speed = 2, leader = 2}
    driver_receive(&d, Driver_Event{kind = .Tick, time = 1, observation = o})
    testing.expect(t, d.actor.state == .Follow_Queue && d.actor.intent == .Pass && d.mode == .Following)
    o.lane, o.filtering = true, true
    driver_receive(&d, Driver_Event{kind = .Maneuver_Result, time = 1, observation = o, feedback = {attempted = true, accepted = true}})
    testing.expect(t, d.actor.state == .Pass && d.mode == .Filtering)
    driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 1, dt = 0.05, observation = o,
        feedback = {reason = .Rear_Gap, blocker = 2}})
    testing.expect(t, d.motion_blocked && d.actor.last.blocker == 2)
    o.speed = 0
    driver_receive(&d, Driver_Event{kind = .Tick, time = 1.05, observation = o})
    testing.expect(t, d.actor.state == .Plan && d.actor.intent == .Recover && d.actor.attempts == 1)
    testing.expect(t, driver_state_parent(d.actor.state) == .Recovery && d.replan_after == 1.55)
    driver_receive(&d, Driver_Event{kind = .Maneuver_Result, time = 1.05, observation = o,
        feedback = {attempted = true, reason = .Search_Limit}})
    testing.expect(t, d.actor.state == .Retry && d.actor.plan.reason == .Search_Limit)
    driver_receive(&d, Driver_Event{kind = .Tick, time = 1.1, observation = o})
    testing.expect(t, d.actor.state == .Retry && d.actor.intent == .Continue && d.actor.attempts == 1,
        "a failed bounded search cannot spin every tick")
    driver_receive(&d, Driver_Event{kind = .Tick, time = 1.55, observation = o})
    testing.expect(t, d.actor.intent == .Recover && d.actor.attempts == 2)
    o.motion, o.bypass, o.lane = true, true, false
    driver_receive(&d, Driver_Event{kind = .Maneuver_Result, time = 1.55, observation = o,
        feedback = {attempted = true, accepted = true, partial = true}})
    testing.expect(t, d.actor.state == .Execute)
    o.speed = 1
    driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 1.55, dt = 0.05, observation = o, feedback = {moved = 0.05}})
    driver_receive(&d, Driver_Event{kind = .Tick, time = 2.1, observation = o})
    testing.expect(t, d.actor.state == .Execute && d.actor.intent == .Continue && d.actor.attempts == 2,
        "a working recovery stays stable even after the retry deadline")
    driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 2.1, observation = o, feedback = {finished = true}})
    driver_receive(&d, Driver_Event{kind = .Tick, time = 3, observation = o})
    testing.expect(t, d.actor.state == .Completed)
}

@(test)
test_actor_no_progress_is_actual_motion_not_a_following_or_signal_label :: proc(t: ^testing.T) {
    d: Driver
    o := Driver_Observation{following = true, can_recover = true, speed = 0}
    for tick in 0..<10 {
        time := f64(tick)*0.05
        driver_receive(&d, Driver_Event{kind = .Tick, time = time, dt = 0.05, observation = o})
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = time, dt = 0.05, observation = o,
            feedback = {moved = 0.001}})
    }
    testing.expect(t, d.actor.no_progress == 0 && d.waited > 0 && d.actor.attempts == 0,
        "a slowly moving queue can build patience without being stuck")
    for tick in 0..<5 {
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 1+f64(tick)*0.05, dt = 0.05, observation = o,
            feedback = {reason = .Following, blocker = 7}})
    }
    testing.expect(t, d.actor.no_progress >= DRIVER_STALL_SECONDS)
    driver_receive(&d, Driver_Event{kind = .Tick, time = 1.3, observation = o})
    testing.expect(t, d.actor.intent == .Recover)
    o.signal_hold = true
    for tick in 0..<100 {
        time := 2+f64(tick)*0.05
        driver_receive(&d, Driver_Event{kind = .Tick, time = time, observation = o})
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = time, dt = 0.05, observation = o,
            feedback = {reason = .Signal}})
        testing.expect(t, d.actor.state == .Wait_Signal && d.actor.intent == .Continue && d.actor.no_progress == 0)
    }
    testing.expect(t, d.actor.attempts == 1, "red waiting cannot create recovery attempts")
    // Commitment, not signal colour, controls junction clearance. A recovery
    // yield must not erase its parent identity or its real blocker feedback.
    o.signal_hold, o.motion, o.bypass = false, true, true
    driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 8, dt = 0.05, observation = o,
        feedback = {reason = .Predicted_Conflict, blocker = 9}})
    testing.expect(t, d.actor.state == .Execute && d.actor.last.blocker == 9)
}

@(test)
test_actor_perception_retains_signal_and_route_boundaries :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{100,0}}, {{200,0}}}, []Road{{0,1,8,9}, {1,2,8,9}, {2,1,8,9}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{node = 1, green = 1, amber = 1, clearance = 100, offset = 2}}, nil))
    d, _ := make_driver(w.behavior, 1, 1, .Motorcycle, .Balanced)
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), id = 1, driver = d})
    o := observe_driver(&w, 0, {})
    testing.expect(t, o.signal_hold && !o.can_recover)
    w.vehicles[0].distance = 20
    o = observe_driver(&w, 0, {})
    testing.expect(t, !o.signal_hold && o.can_recover, "upstream red does not forbid a compliant same-link corridor")
    w.vehicles[0].motion = Connector{active = true, node = 1, exit = 1, lead = 5, progress = 4, length = 20}
    o = observe_driver(&w, 0, {})
    testing.expect(t, !o.signal_hold && !o.can_recover, "a protected crossing lead remains straight")
    w.vehicles[0].motion.progress = 5
    o = observe_driver(&w, 0, {})
    testing.expect(t, o.can_recover, "committed drivers may clear after signal changes")
}

@(test)
test_actor_pass_discovery_retains_the_existing_opportunity_range :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    following: Following_Observation
    v := w.vehicles[1]
    _ = following_acceleration(&w, v, 1, v.driver.traits.headway, v.driver.traits.acceleration, &following)
    testing.expect(t, !following.found, "25m leader is outside this tick's acceleration-relevant contact lookahead")
    o := observe_driver(&w, 1, following)
    testing.expect(t, !o.following && o.pass_candidate, "passing discovery must retain the existing 35m opportunity range")
    driver_receive(&w.vehicles[1].driver, Driver_Event{kind = .Tick, observation = o})
    testing.expect(t, w.vehicles[1].driver.actor.state == .Queue_Approach && w.vehicles[1].driver.actor.intent == .Pass,
        "anticipating a queue is distinct from physical following contact")
    plan: Driver_Feedback
    try_overtake(&w, 1, &plan)
    driver_receive(&w.vehicles[1].driver, Driver_Event{kind = .Maneuver_Result, observation = observe_driver(&w, 1, following), feedback = plan})
    testing.expect(t, plan.accepted && w.vehicles[1].driver.actor.state == .Pass)
    w.vehicles[1].lane = {}
    w.vehicles[1].speed = 0.4
    w.vehicles[0].distance, w.vehicles[0].speed = 34, 0
    _ = following_acceleration(&w, w.vehicles[1], 1, v.driver.traits.headway, v.driver.traits.acceleration, &following)
    o = observe_driver(&w, 1, following)
    testing.expect(t, o.stopped_leader && w.vehicles[1].driver.actor.no_progress == 0)
    driver_receive(&w.vehicles[1].driver, Driver_Event{kind = .Tick, observation = o})
    testing.expect(t, w.vehicles[1].driver.actor.intent == .Recover,
        "an approaching rider can respond to a real stationary obstruction before stopping too")
}
