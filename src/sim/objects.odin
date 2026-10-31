package sim

import "core:math"

finite :: proc(x: f32) -> bool { return !math.is_nan(x) && !math.is_inf(x) }

// Static shortest-hop routes are computed once, in road-index order. There is no
// congestion-aware rerouting; rebuild routes if the road graph changes.
configure :: proc(w: ^World, signals: []Signal_Plan, spawns: []Spawn_Point) -> bool {
    if len(w.vehicles) != 0 || w.time != 0 || w.spawned != 0 || w.completed != 0 || w.requested != 0 { return false }
    for r in w.roads {
        if r.from < 0 || r.from >= len(w.nodes) || r.to < 0 || r.to >= len(w.nodes) || !finite(r.width) { return false }
    }
    for s, i in signals {
        // Timers are bounded for the 20 Hz simulation and practical integer demand counts.
        if s.node < 0 || s.node >= len(w.nodes) || !valid_signal_timings(s) { return false }
        for j in 0..<i { if signals[j].node == s.node { return false } }
        incoming := 0
        for r, road in w.roads {
            if r.to == s.node {
                line := stop_line_for_signals(w, road, signals)
                _, _, _, _, crossing_ok := crossing_band_for_signals(w, road, signals)
                if !crossing_ok { return false }
                if !finite(line) || line <= 0 { return false }
                incoming += 1
            }
        }
        if incoming < 2 { return false }
    }
    runtimes := make([dynamic]Spawn_Runtime)
    success := false
    defer {
        if !success {
            for &runtime in runtimes { delete(runtime.next) }
            delete(runtimes)
        }
    }
    for s in spawns {
        if s.road < 0 || s.road >= len(w.roads) || s.destination < 0 || s.destination >= len(w.nodes) || !finite(s.interval) || s.interval < 0.05 || s.interval > 86400 || int(s.kind) < 0 || int(s.kind) > int(Vehicle_Kind.Tricycle) { return false }
        first := w.roads[s.road]
        if s.destination == first.from || first.width < kind_width(s.kind) { return false }
        next := make([dynamic]int, len(w.nodes))
        for &n in next { n = -1 }
        // Reverse BFS produces an acyclic tree even when the graph contains loops.
        queue := make([dynamic]int)
        append(&queue, s.destination)
        next[s.destination] = -2
        cursor := 0
        for cursor < len(queue) {
            node := queue[cursor]
            cursor += 1
            for r, i in w.roads {
                if r.to == node && r.width >= kind_width(s.kind) && next[r.from] == -1 {
                    next[r.from] = i
                    append(&queue, r.from)
                }
            }
        }
        delete(queue)
        if next[first.to] == -1 { delete(next); return false }
        append(&runtimes, Spawn_Runtime{next = next})
    }
    stop_lines := make([dynamic]f32, len(w.roads))
    // Cache uncontrolled entry boundaries too; all use the same geometry.
    clear(&w.fit_cache)
    clear(&w.road_offsets)
    for _, i in w.roads { append(&w.road_offsets, road_offset(w, i)) }
    for _, road in w.roads { stop_lines[road] = stop_line_for_signals(w, road, signals) }
    new_signals := make([dynamic]Signal_Plan)
    new_spawns := make([dynamic]Spawn_Point)
    for s in signals { append(&new_signals, s) }
    for s in spawns { append(&new_spawns, s) }
    delete(w.signals)
    delete(w.spawns)
    for &runtime in w.spawn_runtime { delete(runtime.next) }
    delete(w.spawn_runtime)
    delete(w.stop_lines)
    w.stop_lines = stop_lines
    w.signals, w.spawns, w.spawn_runtime = new_signals, new_spawns, runtimes
    success = true
    return true
}

Signal_Spec :: struct {
    node: int,
    green, amber, clearance, offset: f32,
    countdown: bool,
}

default_signal_spec :: proc() -> Signal_Spec {
    return {green = 8, amber = 3, clearance = 2}
}

valid_signal_timings :: proc(plan: Signal_Plan) -> bool {
    return finite(plan.green) && finite(plan.amber) && finite(plan.clearance) && finite(plan.offset) &&
        plan.green >= 0.05 && plan.green <= 86400 && plan.amber >= 0.05 && plan.amber <= 86400 &&
        plan.clearance >= 0.05 && plan.clearance <= 86400 && plan.offset >= 0 && plan.offset <= 86400
}

// Value-owned plan from authored timings. Topology (incoming roads, stop lines)
// is checked by configure; flashing yellow and turn-only phases are not options.
make_signal :: proc(spec: Signal_Spec) -> (Signal_Plan, bool) {
    if spec.node < 0 { return {}, false }
    plan := Signal_Plan{
        node = spec.node,
        green = spec.green,
        amber = spec.amber,
        clearance = spec.clearance,
        offset = spec.offset,
        countdown = spec.countdown,
    }
    if !valid_signal_timings(plan) { return {}, false }
    return plan, true
}

signal_countdown_seconds :: proc(remaining: f32) -> int {
    if !finite(remaining) || remaining <= 0 { return 0 }
    return int(math.ceil(remaining))
}

// O(signals*roads) per query is bounded by the small spike maps; cache approach
// indexes before scaling the graph or active population.
signal_phase :: proc(w: ^World, road: int) -> (state: Signal_State, remaining: f32, plan: Signal_Plan, ok: bool) {
    if road < 0 || road >= len(w.roads) { return .Uncontrolled, 0, {}, false }
    for s in w.signals {
        if w.roads[road].to != s.node { continue }
        count, index := 0, 0
        for r, i in w.roads {
            if r.to == s.node {
                if i == road { index = count }
                count += 1
            }
        }
        slot := f64(s.green) + f64(s.amber) + f64(s.clearance)
        cycle := slot * f64(count)
        phase := math.mod(w.time + f64(s.offset), cycle)
        if phase < 0 { phase += cycle }
        start := slot * f64(index)
        until_green := start - phase
        if until_green < 0 { until_green += cycle }
        within := phase - start
        if within >= 0 && within < f64(s.green) {
            return .Green, f32(f64(s.green) - within), s, true
        }
        if within >= f64(s.green) && within < f64(s.green) + f64(s.amber) {
            return .Amber, f32(f64(s.green) + f64(s.amber) - within), s, true
        }
        return .Red, f32(until_green), s, true
    }
    return .Uncontrolled, 0, {}, false
}

signal_state :: proc(w: ^World, road: int) -> Signal_State {
    state, _, _, _ := signal_phase(w, road)
    return state
}

scheduled_spawns :: proc(w: ^World, dt: f32) {
    for s, i in w.spawns {
        runtime := &w.spawn_runtime[i]
        runtime.elapsed += f64(dt)
        arrivals := int(runtime.elapsed / f64(s.interval))
        runtime.elapsed -= f64(arrivals) * f64(s.interval)
        runtime.pending += arrivals
        w.pending += arrivals
        w.requested += arrivals
    }
    // Stable spawn order; a blocked source does not prevent other sources progressing.
    for s, i in w.spawns {
        runtime := &w.spawn_runtime[i]
        for runtime.pending > 0 && len(w.vehicles) < 120 {
            desired := source_lateral(w.roads[s.road].width, s.kind, runtime.admitted)
            candidate := Vehicle{road = s.road, kind = s.kind, spawn_index = i, preferred_lateral = desired}
            // Structural diagnostics must survive even when red/occupied
            // entrance fitting cannot supply a candidate body yet.
            structural := source_problem(w, candidate)
            if structural != .None { report_motion_issue(w, candidate, successor(w, candidate), structural); break }
            lateral, fits := fitting_lateral(w, s.road, s.kind, desired)
            if !fits { break }
            candidate.lateral, candidate.id = lateral, u64(w.spawned+1)
            if !source_supported(w, candidate) { break }
            candidate.driver, _ = make_driver(w.behavior, w.behavior_seed, candidate.id, candidate.kind, spawn_persona(w.behavior, w.behavior_seed, candidate.id))
            append(&w.vehicles, candidate)
            runtime.admitted += 1
            runtime.pending -= 1
            w.pending -= 1
            w.spawned += 1
        }
    }
}

// Width-aware source preferences, not lane changes: cycle only after successful
// admission. Occupancy packing still decides whether the body can actually fit.
// No random draws; presentation changes cannot perturb demand determinism.
source_lateral :: proc(width: f32, kind: Vehicle_Kind, admitted: int) -> f32 {
    if kind != .Motorcycle && kind != .Bicycle { return 0 }
    spacing := kind_width(kind)+0.18
    bands := max(1, int((width+0.18)/spacing))
    return (f32(admitted%bands)-f32(bands-1)*0.5)*spacing
}
