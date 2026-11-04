package sim

import "core:testing"
import "core:fmt"

blocked_turn_world :: proc(w: ^World, blocker: Vehicle_Kind, rotate: bool, initial_speed: f32 = 1) {
    init(w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}, {{0,-60}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {2,1,4,0}, {1,2,4,8},
        {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
    })
    if rotate { for &node in w.nodes { node.position = {-node.position.y, node.position.x} } }
    assert(configure(w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}, {2,0,86400,blocker}}))
    rider := Vehicle{road = 0, distance = entry_distance(w, 0, .Motorcycle), kind = .Motorcycle, id = 1}
    other := Vehicle{road = 2, distance = 57, kind = blocker, spawn_index = 1, id = 2}
    rider.driver, _ = make_driver(w.behavior, 1, rider.id, rider.kind, .Balanced)
    other.driver, _ = make_driver(w.behavior, 1, other.id, other.kind, .Balanced)
    append(&w.vehicles, rider, other)
    w.spawned, w.requested = 2, 2
    assert(acquire_connector(w, 0, 4))
    // Reproduce the normal rolling-gap stop, with manoeuvring room remaining,
    // rather than force the rider bumper-to-bumper with an impossible turn.
    w.vehicles[0].speed = initial_speed
    _, _ = move_vehicle(w, 0, 30)
    w.vehicles[0].speed = 0
    assert(w.vehicles[0].motion.active)
}

// A committed rider meets a stopped body with a free corridor beside it. The
// obstruction must remain in place: waiting for it to disappear is not a fix.
@(test)
test_blocked_turn_steers_around_stopped_body :: proc(t: ^testing.T) {
    for blocker in ([]Vehicle_Kind{.Motorcycle, .Car}) {
        for rotate in ([2]bool{false, true}) {
            for reverse in 0..<2 {
                w, replay: World
                blocked_turn_world(&w, blocker, rotate)
                blocked_turn_world(&replay, blocker, rotate)
                if reverse == 1 {
                    w.vehicles[0], w.vehicles[1] = w.vehicles[1], w.vehicles[0]
                    replay.vehicles[0], replay.vehicles[1] = replay.vehicles[1], replay.vehicles[0]
                }
                rider := 0 if reverse == 0 else 1
                stopped, heading := vehicle_pose(&w, w.vehicles[rider])
                original := w.vehicles[rider].motion
                obstruction, _ := vehicle_pose(&w, w.vehicles[1-rider])
                hull := junction_polygon(&w, original.node)
                cleared, avoided := false, false
                max_deviation: f32
                for tick in 0..<400 {
                    step(&w, 0.05)
                    step(&replay, 0.05)
                    testing.expect(t, len(w.vehicles) == len(replay.vehicles))
                    for v, j in w.vehicles { testing.expect(t, v == replay.vehicles[j], "bypass must replay deterministically") }
                    expect_bodies_clear(t, &w)
                    for v in w.vehicles {
                        if v.id == 2 {
                            p, _ := vehicle_pose(&w, v)
                            testing.expect(t, p == obstruction && v.speed == 0, "the blocker must not move to make the test pass")
                        }
                        if v.id != 1 { continue }
                        p, f := vehicle_pose(&w, v)
                        if tick == 0 {
                            testing.expect(t, vec_distance(stopped, p) < 0.01 && heading.x*f.x+heading.y*f.y > 0.999,
                                "adopting a bypass must not teleport or rotate the body in place")
                        }
                        if v.motion.bypass {
                            avoided = true
                            testing.expect(t, v.motion.exit == original.exit && v.motion.sequence == original.sequence && v.speed <= 2.001)
                        }
                        // Check the executed body, not only the candidate path.
                        for along in ([]f32{-0.5,0,0.5}) { for across in ([]f32{-0.5,0,0.5}) {
                            point := Vec2{p.x+f.x*along*kind_length(v.kind)-f.y*across*kind_width(v.kind),
                                p.y+f.y*along*kind_length(v.kind)+f.x*across*kind_width(v.kind)}
                            testing.expect(t, point_on_junction_pavement(&w, original.node, point, hull[:]))
                        }}
                        if v.motion.active {
                            nearest := f32(1e30)
                            for distance := original.progress; distance <= original.length; distance += 0.1 {
                                q, _ := connector_pose(original, distance)
                                nearest = min(nearest, vec_distance(p, q))
                            }
                            max_deviation = max(max_deviation, nearest)
                        }
                        if v.road == 4 && !v.motion.active { cleared = true }
                    }
                    if cleared { break }
                }
                testing.expect(t, avoided && cleared && len(w.motion_issues) == 0,
                    fmt.tprintf("must steer past the stopped %v within 20s; rotate=%v reverse=%d", blocker, rotate, reverse))
                if blocker == .Motorcycle { testing.expect(t, max_deviation < 2, "the motorcycle case needs only a small lateral slip") }
                delete(hull)
                destroy(&w)
                destroy(&replay)
            }
        }
    }
}

@(test)
test_bypass_rejects_full_width_body_barrier :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{60,0}}, {{120,0}}}, []Road{{0,1,0.8,8}, {1,2,0.8,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    rider := Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), kind = .Motorcycle, id = 1}
    append(&w.vehicles, rider)
    testing.expect(t, acquire_connector(&w, 0, 1))
    append(&w.vehicles, Vehicle{road = 1, distance = 0, kind = .Motorcycle, id = 2})
    original := w.vehicles[0]
    testing.expect(t, !try_local_bypass(&w, 0), "a 0.8m strip has no side corridor past a 0.75m body")
    testing.expect(t, w.vehicles[0].motion == original.motion)
    p, f := vehicle_pose(&w, w.vehicles[0])
    q, g := vehicle_pose(&w, original)
    testing.expect(t, p == q && f == g)
}

@(test)
test_distant_queue_does_not_veto_available_local_escape :: proc(t: ^testing.T) {
    w: World
    blocked_turn_world(&w, .Motorcycle, false)
    defer destroy(&w)
    original := w.vehicles[0]
    // A later obstruction is not a reason to reject the free space beside the
    // immediate blocker. Advance locally and yield again at the actual queue.
    w.behavior.enabled = true
    for side in ([]f32{-1,1}) {
        append(&w.vehicles, Vehicle{road = 4, distance = original.motion.end, lateral = side, kind = .Car,
            driver = {traits = {speed_factor = 1, headway = 0.8, acceleration = 0}}})
    }
    testing.expect(t, try_local_bypass(&w, 0))
    start, _ := vehicle_pose(&w, original)
    advanced := false
    for _ in 0..<200 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        p, _ := vehicle_pose(&w, w.vehicles[0])
        advanced = advanced || vec_distance(start, p) > 2
    }
    testing.expect(t, advanced && w.completed == 0, "take the available escape without driving through the later queue")
}

@(test)
test_bypass_keeps_controlled_crossing_straight :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{0,60}}}, []Road{{0,1,4,8}, {1,2,4,8}, {2,1,4,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0,false}}, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), kind = .Motorcycle, id = 1})
    testing.expect(t, acquire_connector(&w, 0, 1))
    w.vehicles[0].motion.progress = w.vehicles[0].motion.lead*0.5
    original := w.vehicles[0].motion
    testing.expect(t, original.lead > 0)
    obstacle := w.vehicles[0]
    obstacle.id = 2
    obstacle.motion.progress += 2
    append(&w.vehicles, obstacle)
    testing.expect(t, !try_local_bypass(&w, 0) && w.vehicles[0].motion == original,
        "a local bypass cannot start steering on the protected straight crossing lead")
}

// Two valid, committed turns obstruct each other's next few metres. With the
// old fixed guides both stop forever. A roughly one-metre detour lets both go.
@(test)
test_mutually_blocked_motorcycles_take_small_detour_and_drain :: proc(t: ^testing.T) {
    for rotate in ([2]bool{false, true}) {
        for reverse in 0..<2 {
            w: World
            init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}, {{0,-60}}}, []Road{
                {0,1,4,8}, {1,0,4,8}, {2,1,4,8}, {1,2,4,8},
                {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
            })
            if rotate { for &node in w.nodes { node.position = {-node.position.y, node.position.x} } }
            testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}, {2,4,86400,.Motorcycle}}))
            for road, j in ([2]int{0,2}) {
                v := Vehicle{road = road, distance = entry_distance(&w, road, .Motorcycle), kind = .Motorcycle, spawn_index = j, id = u64(j+1)}
                v.driver, _ = make_driver(w.behavior, 1, v.id, v.kind, .Balanced)
                problem: Motion_Problem
                v.motion, problem = make_connector(&w, v, successor(&w, v), 0)
                testing.expect(t, problem == .None)
                v.motion.progress = 4.75 if j == 0 else 10.3
                append(&w.vehicles, v)
            }
            w.spawned, w.requested = 2, 2
            if reverse == 1 { w.vehicles[0], w.vehicles[1] = w.vehicles[1], w.vehicles[0] }
            expect_bodies_clear(t, &w)
            originals := [2]Connector{w.vehicles[0].motion, w.vehicles[1].motion}
            for v, j in w.vehicles {
                blocked := false
                for distance := f32(0.1); distance <= 3; distance += 0.1 {
                    trial := v
                    trial.motion.progress += distance
                    if bodies_overlap(&w, trial, w.vehicles[1-j]) { blocked = true; break }
                }
                testing.expect(t, blocked, "fixture must obstruct both original guides, not just a normal yielding stream")
            }
            avoided := false
            checked: [2]bool
            for _ in 0..<400 {
                step(&w, 0.05)
                expect_bodies_clear(t, &w)
                for v in w.vehicles {
                    avoided = avoided || v.motion.bypass
                    index := int(v.id)-1 if reverse == 0 else 2-int(v.id)
                    if !v.motion.bypass || checked[index] { continue }
                    checked[index] = true
                    original := originals[index]
                    // The rejected old spline is not a corridor constraint.
                    // Require the same exit and a fully fitted replacement,
                    // rather than forcing the escape back toward that spline.
                    testing.expect(t, v.motion.exit == original.exit && v.motion.sequence == original.sequence)
                    hull := junction_polygon(&w, v.motion.node)
                    testing.expect(t, connector_on_pavement(&w, v.motion, v.kind, hull[:]))
                    delete(hull)
                    if !v.motion.partial {
                        p, f := connector_pose(v.motion, v.motion.length)
                        docked := Vehicle{road = v.motion.exit, distance = v.motion.end, lateral = v.motion.lateral, kind = v.kind}
                        q, g := vehicle_pose(&w, docked)
                        testing.expect(t, vec_distance(p, q) < 0.0001 && f.x*g.x+f.y*g.y > 0.99999)
                    }
                }
                if len(w.vehicles) == 0 { break }
            }
            testing.expect(t, avoided && w.completed == 2 && len(w.vehicles) == 0 && len(w.motion_issues) == 0,
                fmt.tprintf("a small available side corridor must drain both riders within 20s; rotate=%v reverse=%d completed=%d remaining=%d", rotate, reverse, w.completed, len(w.vehicles)))
            destroy(&w)
        }
    }
}

@(test)
test_close_obstruction_uses_earlier_steering :: proc(t: ^testing.T) {
    w: World
    blocked_turn_world(&w, .Car, false, 0.2)
    defer destroy(&w)
    cleared := false
    for _ in 0..<400 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        for v in w.vehicles {
            if v.id == 1 && v.road == 4 && !v.motion.active { cleared = true }
            if v.id == 2 { testing.expect(t, v.road == 2 && v.distance == 57 && v.speed == 0) }
        }
        if cleared { break }
    }
    testing.expect(t, cleared, "a close but passable obstruction needs earlier steering, not only a wider destination offset")
}

@(test)
test_partial_steering_cannot_cross_the_receiving_stop_line :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{60,0}}, {{120,0}}}, []Road{{0,1,8,9}, {1,2,8,9}})
    defer destroy(&w)
    origin, axis := vehicle_pose(&w, Vehicle{road = 1})
    maximum := entry_distance(&w, 1, .Motorcycle)
    start, _ := vehicle_pose(&w, Vehicle{road = 1, distance = maximum-0.1})
    hull := junction_polygon(&w, 1)
    defer delete(hull)
    testing.expect(t, !steering_action_clear(&w, 1, 0, 1, axis, .Motorcycle, start, axis, 0, hull[:], nil, 50, origin, axis, maximum),
        "a locally clear partial plan must not cross the next stop line while seeking a future docking pose")
}

@(test)
test_partial_steering_hands_off_an_arrived_receiving_body :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{60,0}}, {{120,0}}}, []Road{{0,1,8,9}, {1,2,8,9}})
    defer destroy(&w)
    minimum := junction_clearance_distance(&w, 1)+kind_length(.Motorcycle)*0.5
    p, f := vehicle_pose(&w, Vehicle{road = 1, distance = kind_length(.Motorcycle)+0.25})
    v := Vehicle{road = 0, kind = .Motorcycle, id = 1}
    v.motion = Connector{active = true, bypass = true, partial = true, node = 1, exit = 1,
        start = {p.x-f.x*STEERING_STEP, p.y-f.y*STEERING_STEP}, forward = f, outgoing = f,
        end = minimum, length = STEERING_STEP, progress = STEERING_STEP, steering_count = 1}
    v.motion.steering_poses[0] = {v.motion.start, f}
    v.motion.steering_poses[1] = {p, f}
    append(&w.vehicles, v)
    testing.expect(t, try_local_bypass(&w, 0))
    q, g := vehicle_pose(&w, w.vehicles[0])
    testing.expect(t, !w.vehicles[0].motion.active && w.vehicles[0].road == 1 && vec_distance(p, q) < 0.0001 && f == g,
        "an aligned body inside its receiving strip must hand off in place, not wait for an obsolete farther marker")
}

@(test)
test_same_link_steering_slips_past_a_stopped_offset_car :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{0,100}}}, []Road{{0,1,4,8}})
    defer destroy(&w)
    w.spawn_accumulator = -1000
    config := default_driver_config()
    config.enabled = true
    testing.expect(t, configure_drivers(&w, config))
    rider, _ := make_driver(config, 1, 2, .Motorcycle, .Balanced)
    append(&w.vehicles,
        Vehicle{road = 0, distance = 23.7, lateral = -0.11, kind = .Car, id = 1},
        Vehicle{road = 0, distance = 20, lateral = -1.395, kind = .Motorcycle, id = 2, driver = rider})
    avoided, passed := false, false
    for _ in 0..<400 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        for v in w.vehicles {
            if v.id == 1 { testing.expect(t, v.distance == 23.7 && v.speed == 0) }
            if v.id != 2 { continue }
            if v.motion.active {
                avoided = avoided || v.motion.local
                testing.expect(t, v.motion.local && v.motion.exit == 0, "link steering must not advance the destination route edge")
            }
            passed = passed || (!v.motion.active && v.distance > 23.7+(kind_length(.Car)+kind_length(.Motorcycle))*0.5)
        }
        if passed { break }
    }
    testing.expect(t, avoided && passed, "a link has the same physical side corridor as a junction; a fixed long lane-change template must not hide it")
}
