// Native, value-owned simulation cases. No renderer state or wall clock enters
// a replay. Static route/geometry caches are rebuilt from their authoritative inputs.
package replay

import "core:encoding/cbor"
import "core:fmt"
import "core:math"
import virtual "core:mem/virtual"
import "core:os"
import sim "../sim"

Runtime :: struct { elapsed: f64, pending, admitted: int }
Snapshot :: struct {
    version: int,
    selected: u64,
    duration: f64,
    min_progress: f32,
    nodes: []sim.Node,
    roads: []sim.Road,
    vehicles: []sim.Vehicle,
    signals: []sim.Signal_Plan,
    spawns: []sim.Spawn_Point,
    behavior: sim.Driver_Config,
    runtimes: []Runtime,
    issues: []sim.Motion_Issue,
    time: f64,
    rng, behavior_seed, entry_sequence: u64,
    spawned, completed, requested, pending: int,
    spawn_accumulator: f32,
}
Report :: struct {
    passed, found, completed: bool,
    progress: f32,
    overlaps, off_pavement: int,
}

capture :: proc(w: ^sim.World, selected: u64, duration: f64, min_progress: f32) -> Snapshot {
    s := Snapshot{version = 2, selected = selected, duration = duration, min_progress = min_progress,
        nodes = w.nodes[:], roads = w.roads[:], vehicles = w.vehicles[:], signals = w.signals[:], spawns = w.spawns[:],
        behavior = w.behavior, issues = w.motion_issues[:], time = w.time, rng = w.rng, behavior_seed = w.behavior_seed,
        entry_sequence = w.entry_sequence, spawned = w.spawned, completed = w.completed, requested = w.requested,
        pending = w.pending, spawn_accumulator = w.spawn_accumulator, runtimes = make([]Runtime, len(w.spawn_runtime))}
    for r, i in w.spawn_runtime { s.runtimes[i] = {r.elapsed, r.pending, r.admitted} }
    return s
}

finite :: proc(x: $T) -> bool { return !math.is_nan(x) && !math.is_inf(x) }

valid_feedback :: proc(f: sim.Driver_Feedback) -> bool {
    return int(f.reason) >= 0 && int(f.reason) <= int(sim.Driver_Block_Reason.Partial_End) &&
        int(f.problem) >= 0 && int(f.problem) <= int(sim.Motion_Problem.Terminal_Clearance) && finite(f.moved) && f.moved >= 0
}

valid :: proc(s: Snapshot) -> bool {
    if (s.version != 1 && s.version != 2) || !finite(s.time) || s.time < 0 || !finite(s.duration) || s.duration <= 0 || s.duration > 300 ||
        !finite(s.min_progress) || s.min_progress <= 0 || s.min_progress > 100 || !finite(s.spawn_accumulator) ||
        len(s.nodes) == 0 || len(s.roads) == 0 || len(s.vehicles) > 120 || len(s.runtimes) != len(s.spawns) ||
        s.spawned != s.completed+len(s.vehicles) || s.completed < 0 || s.pending < 0 || s.requested < 0 { return false }
    if len(s.spawns) > 0 && s.requested != s.spawned+s.pending { return false }
    for n in s.nodes { if !finite(n.position.x) || !finite(n.position.y) { return false } }
    for r in s.roads {
        if r.from < 0 || r.from >= len(s.nodes) || r.to < 0 || r.to >= len(s.nodes) ||
            !finite(r.width) || r.width <= 0 || !finite(r.speed_limit) || r.speed_limit < 0 { return false }
    }
    for r in s.runtimes { if !finite(r.elapsed) || r.elapsed < 0 || r.pending < 0 || r.admitted < 0 { return false } }
    for v, i in s.vehicles {
        if v.road < 0 || v.road >= len(s.roads) || v.id == 0 || int(v.kind) < 0 || int(v.kind) > 5 ||
            !finite(v.distance) || v.distance < 0 || !finite(v.lateral) || !finite(v.preferred_lateral) || !finite(v.speed) || v.speed < 0 ||
            !finite(v.driver.replan_after) || v.driver.replan_after < 0 || !finite(v.driver.waited) || v.driver.waited < 0 ||
            !finite(v.driver.traits.speed_factor) || !finite(v.driver.traits.headway) || !finite(v.driver.traits.acceleration) { return false }
        if s.version == 2 {
            if int(v.driver.mode) < 0 || int(v.driver.mode) > int(sim.Driver_Mode.Avoiding) ||
                int(v.driver.persona) < 0 || int(v.driver.persona) > int(sim.Driver_Persona.Assertive) ||
                int(v.driver.kind) < 0 || int(v.driver.kind) > int(sim.Vehicle_Kind.Tricycle) { return false }
            a := v.driver.actor
            if int(a.state) < 0 || int(a.state) > int(sim.Driver_State.Completed) ||
                int(a.intent) < 0 || int(a.intent) > int(sim.Driver_Intent.Recover) ||
                !finite(a.since) || a.since < 0 || !finite(a.junction_wait) || a.junction_wait < 0 ||
                !valid_feedback(a.last) || !valid_feedback(a.plan) { return false }
        }
        if len(s.spawns) > 0 && (v.spawn_index < 0 || v.spawn_index >= len(s.spawns)) { return false }
        for other in s.vehicles[:i] { if other.id == v.id { return false } }
        if v.lane.active && (!finite(v.lane.start) || !finite(v.lane.from) || !finite(v.lane.to) || !finite(v.lane.length) || v.lane.length <= 0) { return false }
        c := v.motion
        if !c.active { continue }
        if c.local && (c.exit != v.road || c.steering_count == 0) { return false }
        if c.node < 0 || c.node >= len(s.nodes) || c.exit < -1 || c.exit >= len(s.roads) ||
            !finite(c.progress) || !finite(c.length) || c.length <= 0 || c.progress < 0 || c.progress > c.length ||
            !finite(c.end) || !finite(c.lateral) || !finite(c.lead) || !finite(c.tail) || c.lead < 0 || c.tail < 0 { return false }
        for p in ([3]sim.Vec2{c.start, c.forward, c.outgoing}) { if !finite(p.x) || !finite(p.y) { return false } }
        for p in c.controls { if !finite(p.x) || !finite(p.y) { return false } }
        for p in c.continuation { if !finite(p.x) || !finite(p.y) { return false } }
        for d, j in c.arc { if !finite(d) || d < 0 || (j > 0 && d < c.arc[j-1]) { return false } }
        if abs(c.length-c.lead-c.arc[64]-c.tail) > 0.01 { return false }
        if c.steering_count < 0 || c.steering_count > len(c.steering) || (c.partial && c.steering_count == 0) { return false }
        if c.steering_count > 0 {
            if !c.bypass || c.exit < 0 || abs(c.length-f32(c.steering_count)*sim.STEERING_STEP) > 0.0001 { return false }
            if c.lead != 0 || c.tail != 0 || c.steering_poses[0] != ([2]sim.Vec2{c.start, c.forward}) { return false }
            for k in 0..<c.steering_count {
                if !finite(c.steering[k]) || abs(c.steering[k]) > math.acos(f32(0.98))/0.05 { return false }
                previous, next := c.steering_poses[k], c.steering_poses[k+1]
                p, f := sim.curvature_pose(previous[0], previous[1], c.steering[k], sim.STEERING_STEP)
                if sim.vec_distance(p, next[0]) > 0.0001 || sim.vec_distance(f, next[1]) > 0.0001 { return false }
            }
            for pose in c.steering_poses[:c.steering_count+1] {
                for p in pose { if !finite(p.x) || !finite(p.y) { return false } }
                if abs(pose[1].x*pose[1].x+pose[1].y*pose[1].y-1) > 0.0001 { return false }
            }
        }
    }
    return true
}

// Build a replacement completely before touching the caller's live simulation.
restore :: proc(w: ^sim.World, s: Snapshot) -> bool {
    if !valid(s) { return false }
    next: sim.World
    sim.init(&next, s.nodes, s.roads, s.behavior_seed)
    if !sim.configure(&next, s.signals, s.spawns) || !sim.configure_drivers(&next, s.behavior) { sim.destroy(&next); return false }
    for v in s.vehicles {
        c := v.motion
        if !c.active || c.steering_count == 0 || c.partial { continue }
        docked := sim.Vehicle{road = c.exit, distance = c.end, lateral = c.lateral, kind = v.kind}
        p, f := sim.vehicle_pose(&next, docked)
        end := c.steering_poses[c.steering_count]
        if sim.vec_distance(p, end[0]) > 0.0001 || sim.vec_distance(f, end[1]) > 0.0001 {
            sim.destroy(&next)
            return false
        }
    }
    append(&next.vehicles, ..s.vehicles)
    append(&next.motion_issues, ..s.issues)
    for r, i in s.runtimes { next.spawn_runtime[i].elapsed, next.spawn_runtime[i].pending, next.spawn_runtime[i].admitted = r.elapsed, r.pending, r.admitted }
    next.time, next.rng, next.entry_sequence = s.time, s.rng, s.entry_sequence
    next.spawned, next.completed, next.requested, next.pending = s.spawned, s.completed, s.requested, s.pending
    next.spawn_accumulator = s.spawn_accumulator
    if s.version == 1 {
        for &v, i in next.vehicles {
            v.driver.actor = {}
            headway, preferred := f32(0.8), f32(2)
            if next.behavior.enabled { headway, preferred = v.driver.traits.headway, v.driver.traits.acceleration }
            following: sim.Following_Observation
            _ = sim.following_acceleration(&next, v, i, headway, preferred, &following)
            sim.initialize_driver_actor(&v.driver, sim.observe_driver(&next, i, following), next.time)
            v.driver.waited = 0
            sim.publish_driver_mode(&v.driver)
        }
    }
    sim.destroy(w)
    w^ = next
    return true
}

restart :: proc(w: ^sim.World) -> bool {
    next: sim.World
    sim.init(&next, w.nodes[:], w.roads[:], w.behavior_seed)
    if !sim.configure(&next, w.signals[:], w.spawns[:]) || !sim.configure_drivers(&next, w.behavior) { sim.destroy(&next); return false }
    sim.destroy(w)
    w^ = next
    return true
}

clone :: proc(source, destination: ^sim.World) -> bool {
    state := capture(source, 0, 20, 1)
    defer delete(state.runtimes)
    return restore(destination, state)
}

save :: proc(w: ^sim.World, path: string, selected: u64, duration: f64 = 20, min_progress: f32 = 0.5) -> bool {
    state := capture(w, selected, duration, min_progress)
    defer delete(state.runtimes)
    if !valid(state) { return false }
    // CBOR retains exact f32/f64 bits; the std JSON float formatter does not.
    data, err := cbor.marshal(state)
    if err != nil { return false }
    defer delete(data)
    staged := fmt.aprintf("%s.tmp", path)
    defer delete(staged)
    defer os.remove(staged)
    if os.write_entire_file(staged, data) != nil { return false }
    return os.rename(staged, path) == nil
}

// Input is parsed in a private arena and copied into owned simulation storage.
// Failed/partial CBOR never leaks allocations or replaces the current scene.
load_case :: proc(w: ^sim.World, path: string) -> (selected: u64, duration: f64, min_progress: f32, ok: bool) {
    data, err := os.read_entire_file(path, context.allocator)
    if err != nil { return }
    defer delete(data)
    if len(data) > 16*1024*1024 { return }
    arena: virtual.Arena
    if virtual.arena_init_growing(&arena) != nil { return }
    defer virtual.arena_destroy(&arena)
    state: Snapshot
    if cbor.unmarshal(data, &state, allocator = virtual.arena_allocator(&arena)) != nil { return }
    if !restore(w, state) { return }
    return state.selected, state.duration, state.min_progress, true
}

load :: proc(w: ^sim.World, path: string) -> (selected: u64, ok: bool) {
    selected, _, _, ok = load_case(w, path)
    return
}

// Check a clone, not the editor's live state. A recorded case is a real
// assertion: a selected trip must advance or finish, without overlapping bodies
// or leaving the fitted junction asphalt. All surrounding traffic is retained.
check :: proc(w: ^sim.World, selected: u64, duration: f64 = 20, min_progress: f32 = 0.5) -> Report {
    report: Report
    if !finite(duration) || duration <= 0 || duration > 300 || !finite(min_progress) || min_progress <= 0 { return report }
    start: sim.Vec2
    for v in w.vehicles { if v.id == selected { report.found = true; start, _ = sim.vehicle_pose(w, v); break } }
    if !report.found { return report }
    copy: sim.World
    if !clone(w, &copy) { return {} }
    defer sim.destroy(&copy)
    hulls := make([][dynamic]sim.Vec2, len(copy.nodes))
    for _, node in copy.nodes { hulls[node] = sim.junction_polygon(&copy, node) }
    defer { for h in hulls { delete(h) }; delete(hulls) }
    ticks := int(math.ceil(duration/0.05))
    for tick in 0..=ticks {
        present := false
        for a, i in copy.vehicles {
            for b in copy.vehicles[i+1:] { if sim.bodies_overlap(&copy, a, b, 0) { report.overlaps += 1 } }
            p, f := sim.vehicle_pose(&copy, a)
            external_exit := a.motion.exit < 0 && sim.terminal_is_external(&copy, a.road)
            if a.motion.active && !external_exit {
                hull, permitted := hulls[a.motion.node][:], -1
                if a.motion.local { hull, permitted = nil, a.road }
                if !sim.body_on_junction_pavement(&copy, a.motion.node, p, f, a.kind, hull, permitted, permitted) { report.off_pavement += 1 }
            }
            if a.id == selected { present = true; report.progress = max(report.progress, sim.vec_distance(start, p)) }
        }
        report.completed = report.completed || !present
        if tick < ticks { sim.step(&copy, 0.05) }
    }
    report.passed = (report.completed || report.progress >= min_progress) && report.overlaps == 0 && report.off_pavement == 0
    return report
}
