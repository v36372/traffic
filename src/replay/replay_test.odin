package replay

import "core:fmt"
import "core:os"
import "core:testing"
import sim "../sim"

replay_world :: proc(w: ^sim.World) {
    sim.init(w, []sim.Node{{{0,0}}, {{120,0}}, {{240,0}}}, []sim.Road{{0,1,8,9}, {1,2,8,9}}, 17)
    assert(sim.configure(w, nil, []sim.Spawn_Point{{0,2,0.8,.Motorcycle}}))
    config := sim.default_driver_config()
    config.enabled = true
    config.classes[int(sim.Vehicle_Kind.Motorcycle)] = {0.7,1,0.5,1.2}
    assert(sim.configure_drivers(w, config))
    for _ in 0..<300 { sim.step(w, 0.05) }
}

@(test)
test_snapshot_roundtrip_replays_exact_native_state :: proc(t: ^testing.T) {
    w, restored: sim.World
    replay_world(&w)
    defer sim.destroy(&w)
    defer sim.destroy(&restored)
    dir, err := os.make_directory_temp("", "traffic-replay-*", context.allocator)
    testing.expect(t, err == nil)
    if err != nil { return }
    defer delete(dir)
    defer os.remove(dir)
    path := fmt.aprintf("%s/case.cbor", dir)
    defer delete(path)
    defer os.remove(path)
    testing.expect(t, save(&w, path, w.vehicles[0].id, 7, 0.75))
    selected, duration, progress, ok := load_case(&restored, path)
    testing.expect(t, ok && selected == w.vehicles[0].id && duration == 7 && progress == 0.75)
    if !ok { return }
    for _ in 0..<300 {
        testing.expect(t, w.time == restored.time && w.rng == restored.rng && w.entry_sequence == restored.entry_sequence)
        testing.expect(t, w.spawned == restored.spawned && w.completed == restored.completed && w.requested == restored.requested && w.pending == restored.pending)
        testing.expect(t, len(w.vehicles) == len(restored.vehicles))
        for v, i in w.vehicles {
            testing.expect(t, v == restored.vehicles[i], "snapshot must retain actual maneuvers, drivers, and arbitration order")
            if v != restored.vehicles[i] { return }
        }
        for r, i in w.spawn_runtime {
            other := restored.spawn_runtime[i]
            testing.expect(t, r.elapsed == other.elapsed && r.pending == other.pending && r.admitted == other.admitted)
        }
        sim.step(&w, 0.05)
        sim.step(&restored, 0.05)
    }
    testing.expect(t, save(&w, path, selected), "atomic replacement of an existing saved case must work")
    before := restored.vehicles[0]
    for invalid in ([]string{"", "{", "corrupt case"}) {
        testing.expect(t, os.write_entire_file(path, invalid) == nil)
        _, loaded := load(&restored, path)
        testing.expect(t, !loaded && restored.vehicles[0] == before, "bad snapshots must leave the live scene intact")
    }
}

@(test)
test_case_check_is_a_nonmutating_progress_assertion :: proc(t: ^testing.T) {
    w: sim.World
    sim.init(&w, []sim.Node{{{0,0}}, {{300,0}}}, []sim.Road{{0,1,8,9}}, 1)
    defer sim.destroy(&w)
    w.spawn_accumulator = -1000
    driver, _ := sim.make_driver(w.behavior, 1, 1, .Motorcycle, .Balanced)
    append(&w.vehicles, sim.Vehicle{road = 0, distance = 20, id = 1, kind = .Motorcycle, speed = 2, driver = driver})
    w.spawned = 1
    original := w.vehicles[0]
    result := check(&w, 1, 1, 1)
    testing.expect(t, result.passed && result.found && result.progress >= 1 && result.overlaps == 0 && result.off_pavement == 0)
    testing.expect(t, w.time == 0 && w.vehicles[0] == original, "checking a case must not advance the editor's world")
    w.behavior.enabled = true
    w.vehicles[0].speed = 0
    w.vehicles[0].driver.traits.acceleration = 0
    stopped := check(&w, 1, 1, 0.75)
    testing.expect(t, !stopped.passed && stopped.found && stopped.progress == 0 && stopped.overlaps == 0)
    absent := check(&w, 999, 1, 1)
    testing.expect(t, !absent.passed && !absent.found)
    duplicate := w.vehicles[0]
    duplicate.id = 2
    append(&w.vehicles, duplicate)
    w.spawned += 1
    collision := check(&w, 1, 0.05, 0.1)
    testing.expect(t, !collision.passed && collision.overlaps > 0, "progress cannot hide an unsafe replay")
}

@(test)
test_native_scene_probe_finds_real_available_motion_without_mutation :: proc(t: ^testing.T) {
    w: sim.World
    defer sim.destroy(&w)
    selected, ok := load(&w, "test-cases/default-250-driver98.cbor")
    testing.expect(t, ok && selected == 98 && len(w.vehicles) == 120)
    if !ok { return }
    before := make([]sim.Vehicle, len(w.vehicles))
    defer delete(before)
    copy(before, w.vehicles[:])
    time, rng := w.time, w.rng
    probe, available := steering_probe(&w, selected)
    testing.expect(t, available && probe.distance == 0.75, "the recorded default-scene driver has a real forward option")
    if available {
        selected_vehicle := sim.Vehicle{}
        selected_index: int
        for v, i in w.vehicles { if v.id == selected { selected_vehicle, selected_index = v, i } }
        following: sim.Following_Observation
        _ = sim.following_acceleration(&w, selected_vehicle, selected_index, selected_vehicle.driver.traits.headway,
            selected_vehicle.driver.traits.acceleration, &following)
        testing.expect(t, probe.curvature == 0 && (!following.found || following.gap >= probe.distance),
            "a clear straight probe must not have a fictional following contact inside it")
        hull := sim.junction_polygon(&w, selected_vehicle.motion.node)
        defer delete(hull)
        for i in 0..=15 {
            p, f := probe_pose(probe, f32(i)*0.05)
            testing.expect(t, sim.body_on_junction_pavement(&w, selected_vehicle.motion.node, p, f, selected_vehicle.kind, hull[:]))
            for other in w.vehicles {
                if other.id == selected { continue }
                q, g := sim.vehicle_pose(&w, other)
                testing.expect(t, !sim.body_poses_overlap(p, f, selected_vehicle.kind, q, g, other.kind))
            }
        }
        end, _ := probe_pose(probe, probe.distance)
        testing.expect(t, sim.vec_distance(probe.start, end) > 0.5)
    }
    testing.expect(t, w.time == time && w.rng == rng)
    for v, i in w.vehicles { testing.expect(t, v == before[i]) }
}

@(test)
test_recorded_default_250s_driver_uses_available_forward_space :: proc(t: ^testing.T) {
    w: sim.World
    defer sim.destroy(&w)
    selected, duration, minimum, loaded := load_case(&w, "test-cases/default-250-driver98.cbor")
    testing.expect(t, loaded)
    if !loaded { return }
    for id in ([6]u64{98, 100, 119, 142, 158, 184}) {
        _, available := steering_probe(&w, id)
        testing.expect(t, available, "the real-scene crossing driver has clear forward space")
        required := f32(5) if id == selected else minimum
        result := check(&w, id, duration, required)
        testing.expect(t, result.passed, fmt.tprintf("recorded driver %d must clear the obstruction with all120 vehicles retained: progress=%gm overlaps=%d offroad=%d", id, result.progress, result.overlaps, result.off_pavement))
    }
}

@(test)
test_recorded_near_exit_driver_can_maneuver_beyond_old_transition :: proc(t: ^testing.T) {
    w: sim.World
    defer sim.destroy(&w)
    selected, duration, minimum, loaded := load_case(&w, "test-cases/default-250-near-exit.cbor")
    testing.expect(t, loaded && selected == 184)
    if !loaded { return }
    for v in w.vehicles {
        if v.id == selected { testing.expect(t, v.motion.length-v.motion.progress < 0.1, "reproduce the old end-of-guide refusal") }
    }
    _, available := steering_probe(&w, selected, 1.5)
    testing.expect(t, available, "the captured queue has 1.5m of real steering space")
    result := check(&w, selected, duration, minimum)
    testing.expect(t, result.passed, fmt.tprintf("a transition marker cannot prevent useful steering: progress=%gm overlaps=%d offroad=%d", result.progress, result.overlaps, result.off_pavement))
}

@(test)
test_free_probe_is_not_a_same_exit_route_certificate :: proc(t: ^testing.T) {
    w: sim.World
    defer sim.destroy(&w)
    selected, loaded := load(&w, "test-cases/default-250-probe-limit.cbor")
    testing.expect(t, loaded && selected == 215)
    if !loaded { return }
    probe, available := steering_probe(&w, selected, 3)
    testing.expect(t, available)
    if !available { return }
    for v in w.vehicles {
        if v.id != selected { continue }
        end, heading := probe_pose(probe, probe.distance)
        origin, axis := sim.vehicle_pose(&w, sim.Vehicle{road = v.motion.exit})
        lateral := -(end.x-origin.x)*axis.y+(end.y-origin.y)*axis.x
        // This free arc ends in the opposing strip, heading away from the
        // intended exit. It cannot justify requiring a legal complete pass.
        testing.expect(t, abs(lateral) > w.roads[v.motion.exit].width*0.5)
        testing.expect(t, heading.x*axis.x+heading.y*axis.y < 0)
    }
}

@(test)
test_recorded_receiving_queue_driver_plans_past_the_blocker :: proc(t: ^testing.T) {
    w: sim.World
    defer sim.destroy(&w)
    selected, duration, minimum, loaded := load_case(&w, "test-cases/default-250-receiving-queue.cbor")
    testing.expect(t, loaded && selected == 66 && len(w.vehicles) == 120 && duration == 20 && minimum == 5)
    if !loaded { return }
    rider, blocker: sim.Vehicle
    for v in w.vehicles {
        if v.id == selected { rider = v }
        if v.id == 60 { blocker = v }
    }
    p, f := sim.vehicle_pose(&w, rider)
    origin, axis := sim.vehicle_pose(&w, sim.Vehicle{road = rider.motion.exit})
    hull := sim.junction_polygon(&w, rider.motion.node)
    defer delete(hull)
    // Independent forward-arc witness from the actual scene: a 20cm sideways
    // adjustment passes the stationary body without entering the opposing strip.
    // The production planner does not consume this path or these actions.
    for curvature in ([28]f32{0,-0.25,0,0,0,0,0,0,0,0,0,0,0.125,0,0,0,0.125,0,0,0,0,0,0,0,0,0,0,0}) {
        arc := Steering_Probe{p, f, curvature, 0.25}
        for sample in 0..=5 {
            q, g := probe_pose(arc, f32(sample)*0.05)
            testing.expect(t, sim.body_on_junction_pavement(&w, rider.motion.node, q, g, rider.kind, hull[:]))
            lateral := -(q.x-origin.x)*axis.y+(q.y-origin.y)*axis.x
            side_extent := (abs(g.x*axis.y-g.y*axis.x)*sim.kind_length(rider.kind)+abs(g.x*axis.x+g.y*axis.y)*sim.kind_width(rider.kind))*0.5
            testing.expect(t, abs(lateral)+side_extent <= w.roads[rider.motion.exit].width*0.5+0.0001,
                "the witness stays in the same-direction carriageway")
            for other in w.vehicles {
                if other.id == selected { continue }
                op, of := sim.vehicle_pose(&w, other)
                testing.expect(t, !sim.body_poses_overlap(q, g, rider.kind, op, of, other.kind))
            }
        }
        p, f = probe_pose(arc, arc.distance)
    }
    obstacle, _ := sim.vehicle_pose(&w, blocker)
    testing.expect(t, (p.x-obstacle.x)*axis.x+(p.y-obstacle.y)*axis.y > (sim.kind_length(rider.kind)+sim.kind_length(blocker.kind))*0.5)
    result := check(&w, selected, duration, minimum)
    testing.expect(t, result.passed, fmt.tprintf("the same-direction side corridor must be used: progress=%gm overlaps=%d offroad=%d", result.progress, result.overlaps, result.off_pavement))
    // The maneuver must also exist with every other body held at its captured
    // pose; progress cannot pass merely because the surrounding queue moved.
    for v, i in w.vehicles {
        if v.id != selected { continue }
        before, heading := sim.vehicle_pose(&w, v)
        w.vehicles[i].driver.replan_after = 0
        planned := sim.try_local_bypass(&w, i)
        if planned && !w.vehicles[i].motion.active {
            // Aligned receiving bodies hand off first; the same frozen-scene
            // search must then find the link escape, without moving any body.
            w.vehicles[i].driver.replan_after = 0
            planned = sim.try_local_bypass(&w, i)
        }
        testing.expect(t, planned, "the planner must find the small available sidestep")
        if !planned { break }
        after, forward := sim.vehicle_pose(&w, w.vehicles[i])
        testing.expect(t, before == after && heading == forward, "adopting a path cannot teleport or rotate the body")
        testing.expect(t, w.vehicles[i].motion.active && w.vehicles[i].motion.exit == v.motion.exit)
        testing.expect(t, w.vehicles[i].motion.end <= sim.entry_distance(&w, v.motion.exit, v.kind))
    }
}
