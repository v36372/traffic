package replay

import "core:fmt"
import "core:slice"
import "core:testing"
import sim "../sim"

@(test)
test_captured_stalls_have_independent_space_and_must_progress_safely :: proc(t: ^testing.T) {
    cases := [3]string{
        "test-cases/captured-first-stop-t120.90-driver109.cbor",
        "test-cases/captured-first-stop-t189.30-driver199.cbor",
        "test-cases/captured-first-stop-t218.65-driver184.cbor",
    }
    for path in cases {
        w: sim.World
        selected, loaded := load(&w, path)
        testing.expect(t, loaded, fmt.tprintf("load preserved capture %s", path))
        if !loaded { sim.destroy(&w); continue }
        before := make([]sim.Vehicle, len(w.vehicles))
        defer delete(before)
        copy(before, w.vehicles[:])
        probe, available := steering_probe(&w, selected, 0.75)
        testing.expect(t, available && probe.distance == 0.75,
            fmt.tprintf("driver %d must independently have a clear 0.75m arc", selected))
        result := check(&w, selected, 20, 0.5)
        testing.expect(t, result.passed && result.progress >= 0.5 && result.overlaps == 0 && result.off_pavement == 0,
            fmt.tprintf("captured driver %d must advance >=0.5m in20s with all traffic: progress=%gm overlaps=%d offroad=%d",
                selected, result.progress, result.overlaps, result.off_pavement))
        testing.expect(t, len(w.vehicles) == len(before), "checking must preserve every captured traffic body")
        for v, i in w.vehicles { testing.expect(t, v == before[i], "checking must preserve each captured vehicle state") }
        sim.destroy(&w)
    }
}

@(test)
test_steering_snapshot_rejects_discontinuous_action_boundaries :: proc(t: ^testing.T) {
    w: sim.World
    sim.init(&w, []sim.Node{{{0,0}}, {{100,0}}, {{200,0}}}, []sim.Road{{0,1,8,9}, {1,2,8,9}})
    defer sim.destroy(&w)
    v := sim.Vehicle{road = 0, distance = 80, kind = .Motorcycle, id = 1}
    p, f := sim.vehicle_pose(&w, v)
    c := sim.Connector{active = true, bypass = true, partial = true, node = 1, exit = 1,
        start = p, forward = f, outgoing = f, steering_count = 2, length = 0.5}
    c.steering[0], c.steering[1] = 0.25, -0.25
    c.steering_poses[0] = {p, f}
    for k in 0..<c.steering_count {
        p, f = sim.curvature_pose(p, f, c.steering[k], sim.STEERING_STEP)
        c.steering_poses[k+1] = {p, f}
    }
    for k in 0..<len(c.arc) { c.arc[k] = f32(min(k, c.steering_count))*sim.STEERING_STEP }
    v.motion = c
    append(&w.vehicles, v)
    w.spawned, w.requested = 1, 1
    snapshot := capture(&w, 1, 20, 0.5)
    defer delete(snapshot.runtimes)
    copies := make([]sim.Vehicle, 1)
    defer delete(copies)
    copies[0] = v
    snapshot.vehicles = copies
    testing.expect(t, valid(snapshot))
    copies[0].motion.steering_poses[1][0].x += 1
    testing.expect(t, !restore(&w, snapshot) && w.vehicles[0] == v,
        "finite but discontinuous cached poses must not teleport a loaded driver")
    copies[0] = v
    copies[0].motion.steering_poses[1][1] = {2,0}
    testing.expect(t, !valid(snapshot), "heading vectors must be unit length and match their executed action")
    copies[0] = v
    copies[0].motion.partial = false
    copies[0].motion.end = 50
    testing.expect(t, !restore(&w, snapshot) && w.vehicles[0] == v,
        "the complete path endpoint must match the receiving-road handoff, not teleport to its metadata")
}

@(test)
test_probe_covers_stopped_link_drivers_without_crossing_the_entry_gate :: proc(t: ^testing.T) {
    w: sim.World
    sim.init(&w, []sim.Node{{{0,0}}, {{100,0}}}, []sim.Road{{0,1,8,9}})
    defer sim.destroy(&w)
    append(&w.vehicles, sim.Vehicle{road = 0, distance = 20, kind = .Motorcycle, id = 1})
    probe, available := steering_probe(&w, 1)
    testing.expect(t, available && probe.curvature == 0,
        "the temporal audit must not lose sight of a stopped driver after junction handoff")
    w.vehicles[0].distance = sim.entry_distance(&w, 0, .Motorcycle)-0.1
    _, available = steering_probe(&w, 1)
    testing.expect(t, !available, "link clearance is not permission to bypass junction admission or a signal")
}

@(test)
test_captured_link_stalls_use_the_same_safe_steering_search :: proc(t: ^testing.T) {
    for path in ([]string{"test-cases/captured-link-stop-t185.75-driver192.cbor", "test-cases/captured-link-stop-t227.25-driver260.cbor"}) {
        w: sim.World
        selected, ok := load(&w, path)
        testing.expect(t, ok)
        if !ok { continue }
        for v in w.vehicles { if v.id == selected { testing.expect(t, !v.motion.active && v.speed == 0) } }
        _, available := steering_probe(&w, selected)
        testing.expect(t, available, "the same-link queue has independent local steering space")
        result := check(&w, selected, 20, 5)
        testing.expect(t, result.passed, fmt.tprintf("link driver%d must pass rather than wait behind an offset body: %gm overlap%d offroad%d", selected, result.progress, result.overlaps, result.off_pavement))
        sim.destroy(&w)
    }
}

@(test)
test_captured_lane_veto_replans_instead_of_waiting_in_free_space :: proc(t: ^testing.T) {
    w: sim.World
    selected, ok := load(&w, "test-cases/captured-lane-stop-t168.00-driver211.cbor")
    testing.expect(t, ok)
    if !ok { return }
    defer sim.destroy(&w)
    for v in w.vehicles { if v.id == selected { testing.expect(t, v.lane.active && !v.motion.active && v.speed == 0) } }
    _, available := steering_probe(&w, selected, 3)
    testing.expect(t, available, "three metres straight from the actual pose are independently clear")
    result := check(&w, selected, 3, 0.5)
    testing.expect(t, result.passed, fmt.tprintf("a lane safety veto must feed back into path choice, not ten seconds of zero movement: %gm overlap%d offroad%d", result.progress, result.overlaps, result.off_pavement))
}

@(test)
test_free_local_arc_does_not_certify_a_packed_queue_pass :: proc(t: ^testing.T) {
    w: sim.World
    selected, ok := load(&w, "test-cases/captured-full-queue-t600-driver177.cbor")
    testing.expect(t, ok)
    if !ok { return }
    defer sim.destroy(&w)
    own: sim.Vehicle
    for v in w.vehicles { if v.id == selected { own = v } }
    _, available := steering_probe(&w, selected, 0.75)
    testing.expect(t, available)
    // At this forward cross-section every possible heading needs at least
    // kind_width of room. Frozen, aligned bodies leave no such gap. Ignore
    // the SAT margin: even an unrealistically zero-margin pass cannot fit.
    origin, axis := sim.vehicle_pose(&w, sim.Vehicle{road = own.road})
    plane := own.distance+sim.kind_length(own.kind)
    intervals: [dynamic][2]f32
    defer delete(intervals)
    for v in w.vehicles {
        if v.id == selected || v.road != own.road || v.speed != 0 { continue }
        p, f := sim.vehicle_pose(&w, v)
        if f != axis { continue }
        along := (p.x-origin.x)*axis.x+(p.y-origin.y)*axis.y
        if abs(along-plane) > sim.kind_length(v.kind)*0.5 { continue }
        lateral := -(p.x-origin.x)*axis.y+(p.y-origin.y)*axis.x
        append(&intervals, [2]f32{lateral-sim.kind_width(v.kind)*0.5, lateral+sim.kind_width(v.kind)*0.5})
    }
    slice.sort_by(intervals[:], proc(a, b: [2]f32) -> bool { return a[0] < b[0] })
    half_width := w.roads[own.road].width*0.5
    cursor, widest := -half_width, f32(0)
    for interval in intervals {
        widest = max(widest, clamp(interval[0], -half_width, half_width)-cursor)
        cursor = max(cursor, min(half_width, interval[1]))
    }
    widest = max(widest, half_width-cursor)
    testing.expect(t, widest < sim.kind_width(own.kind), fmt.tprintf("queue gap %gm cannot fit a %gm body", widest, sim.kind_width(own.kind)))
}
