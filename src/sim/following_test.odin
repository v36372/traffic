package sim

import "core:testing"

@(test)
test_blocked_passing_matches_slow_leader_without_bumper_chasing :: proc(t: ^testing.T) {
    for reverse in 0..<2 {
        w: World
        init(&w, []Node{{{0,0}}, {{600,0}}}, []Road{{0,1,0.9,6}})
        defer destroy(&w)
        config := default_driver_config(); config.enabled = true
        config.classes[0] = {0.5,1,0.8,0.8}
        testing.expect(t, configure_drivers(&w, config))
        w.spawn_accumulator = -1000
        lead := Vehicle{road = 0, distance = 30, kind = .Motorcycle, id = 1, spawn_index = -1,
            driver = {traits = {0.5,0.8,1.4}}}
        follower := Vehicle{road = 0, distance = 27.5, kind = .Motorcycle, id = 2, spawn_index = -1,
            driver = {traits = {1,0.8,2.6}}}
        append(&w.vehicles, lead, follower)
        li, fi := 0, 1
        if reverse == 1 { w.vehicles[0], w.vehicles[1] = follower, lead; li, fi = 1, 0 }
        started := false
        for tick in 0..<600 {
            step(&w, 0.05)
            a, b := w.vehicles[li], w.vehicles[fi]
            testing.expect(t, !b.lane.active && !bodies_overlap(&w, a, b))
            if b.speed > 0.3 { started = true }
            if started { testing.expect(t, b.speed > 0.1, "a moving leader must not make a following rider repeatedly halt") }
            gap := a.distance-b.distance-(kind_length(a.kind)+kind_length(b.kind))*0.5
            testing.expect(t, gap > 0.2, "follow at a usable gap instead of catching the emergency movement clamp")
            if tick >= 400 { testing.expect(t, abs(a.speed-b.speed) < 0.15, "blocked passing should settle to the leader's speed") }
        }
        testing.expect(t, started)
        w.vehicles[li].driver.traits.speed_factor = 1
        for _ in 0..<200 { step(&w, 0.05); expect_bodies_clear(t, &w) }
        testing.expect(t, w.vehicles[fi].speed > 4, "accelerate again when the leader opens space")
    }
}

@(test)
test_slow_junction_stream_keeps_rolling_instead_of_halting :: proc(t: ^testing.T) {
    for reverse in 0..<2 {
        w: World
        init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}}, []Road{{0,1,3.5,1}, {1,2,3.5,1}})
        defer destroy(&w)
        config := default_driver_config(); config.enabled = true
        testing.expect(t, configure_drivers(&w, config))
        testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
        entry := entry_distance(&w, 0, .Motorcycle)
        lead := Vehicle{road = 0, distance = entry, speed = 0.5, kind = .Motorcycle, id = 1,
            driver = {traits = {0.5,0.8,1.4}}}
        follower := lead
        follower.id, follower.driver.traits = 2, {1,0.8,2.6}
        c, problem := make_connector(&w, lead, 1, 0)
        testing.expect(t, problem == .None)
        lead.motion, follower.motion = c, c
        lead.motion.progress, follower.motion.progress = 4, 1.5
        lead.motion.sequence, follower.motion.sequence = 1, 2
        w.entry_sequence = 2
        append(&w.vehicles, lead, follower)
        w.spawned, w.requested = 2, 2
        li, fi := 0, 1
        if reverse == 1 { w.vehicles[0], w.vehicles[1] = follower, lead; li, fi = 1, 0 }
        for tick in 0..<300 {
            previous := w.vehicles[fi].speed
            step(&w, 0.05)
            a, b := w.vehicles[li], w.vehicles[fi]
            expect_bodies_clear(t, &w)
            testing.expect(t, b.speed > 0.1, "a rolling junction queue must not be stopped by a fictitious 1m/s forecast")
            testing.expect(t, b.speed >= previous-4*0.05-0.001, "normal following must brake before a hard collision stop")
            if tick >= 200 { testing.expect(t, abs(a.speed-b.speed) < 0.15) }
        }
    }
}
