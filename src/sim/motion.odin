package sim

import "core:math"
import "core:fmt"

// A fitted steering guide, not exclusive occupancy of future pavement.
Connector :: struct {
    active: bool,
    sequence: u64,
    node, exit: int,
    end, lateral, progress, length: f32,
    start, forward: Vec2,
    lead, tail: f32,
    outgoing: Vec2,
    controls: [4]Vec2,
    arc: [65]f32,
}
Motion_Problem :: enum { None, Degenerate, Short_Approach, Short_Exit, U_Turn, Tight_Turn, Terminal_Clearance }
Motion_Issue :: struct { road, exit: int, problem: Motion_Problem }
Motion_Key :: struct { road, exit: int, kind: Vehicle_Kind, start, incoming, outgoing: f32, continues: bool }
Motion_Fit :: struct { connector: Connector, problem: Motion_Problem }

report_motion_issue :: proc(w: ^World, v: Vehicle, exit: int, problem: Motion_Problem) {
    issue := Motion_Issue{v.road, exit, problem}
    for old in w.motion_issues { if old == issue { return } }
    append(&w.motion_issues, issue)
    candidates := connector_laterals(w, v, exit)
    defer delete(candidates)
    fmt.eprintf("sim: unsupported transition %d -> %d: %v; kind=%v incoming=%g preferred=%g receiving_trials=%v; held upstream\n",
        v.road, exit, problem, v.kind, v.lateral, v.preferred_lateral, candidates[:])
}

vec_distance :: proc(a, b: Vec2) -> f32 { return math.sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)) }
bezier_pose :: proc(c: Connector, t: f32) -> (p, f: Vec2) {
    s := 1-t
    a, b, d, e := c.controls[0], c.controls[1], c.controls[2], c.controls[3]
    p = {s*s*s*a.x+3*s*s*t*b.x+3*s*t*t*d.x+t*t*t*e.x,
         s*s*s*a.y+3*s*s*t*b.y+3*s*t*t*d.y+t*t*t*e.y}
    f = {3*s*s*(b.x-a.x)+6*s*t*(d.x-b.x)+3*t*t*(e.x-d.x),
         3*s*s*(b.y-a.y)+6*s*t*(d.y-b.y)+3*t*t*(e.y-d.y)}
    n := math.sqrt(f.x*f.x+f.y*f.y)
    if n > 0 { f.x /= n; f.y /= n }
    return
}
connector_pose :: proc(c: Connector, distance: f32) -> (p, f: Vec2) {
    d := clamp(distance, 0, c.length)
    if d <= c.lead { return Vec2{c.start.x+c.forward.x*d, c.start.y+c.forward.y*d}, c.forward }
    d -= c.lead
    if d >= c.arc[64] {
        tail := d-c.arc[64]
        q := c.controls[3]
        return Vec2{q.x+c.outgoing.x*tail, q.y+c.outgoing.y*tail}, c.outgoing
    }
    for i in 1..<len(c.arc) {
        if d <= c.arc[i] {
            t := (f32(i-1)+(d-c.arc[i-1])/max(0.000001, c.arc[i]-c.arc[i-1]))/64
            return bezier_pose(c, t)
        }
    }
    return bezier_pose(c, 1)
}

successor :: proc(w: ^World, v: Vehicle) -> int {
    if len(w.spawns) == 0 { return next_road(w, v.road) }
    s := w.spawns[v.spawn_index]
    if w.roads[v.road].to == s.destination { return -1 }
    return w.spawn_runtime[v.spawn_index].next[w.roads[v.road].to]
}

// Shared geometry authority: use the same hull/strip setback as signal bars,
// even at uncontrolled junctions. The center waits with its front behind it.
entry_distance :: proc(w: ^World, road: int, kind: Vehicle_Kind) -> f32 {
    if len(w.stop_lines) == len(w.roads) { return w.stop_lines[road]-kind_length(kind)*0.5 }
    return stop_line_distance(w, road)-kind_length(kind)*0.5
}

make_connector :: proc(w: ^World, v: Vehicle, exit: int, lateral: f32) -> (c: Connector, problem: Motion_Problem) {
    continues := false
    if exit >= 0 { next := v; next.road = exit; continues = successor(w, next) >= 0 }
    key := Motion_Key{v.road, exit, v.kind, max(v.distance, entry_distance(w, v.road, v.kind)), v.lateral, lateral, continues}
    if cached, ok := w.fit_cache[key]; ok { return cached.connector, cached.problem }
    c, problem = fit_connector(w, v, exit, lateral)
    // Static geometry trials are reused by pending demand and queued vehicles.
    // Cache grows with distinct lateral trials until configure/reset; precompile
    // movement corridors if city-scale long-lived runs make that a memory limit.
    w.fit_cache[key] = Motion_Fit{c, problem}
    return
}

fit_connector :: proc(w: ^World, v: Vehicle, exit: int, lateral: f32) -> (c: Connector, problem: Motion_Problem) {
    _, f := vehicle_pose(w, v)
    g := f
    if exit >= 0 { _, g = vehicle_pose(w, Vehicle{road = exit}) }
    left := f.x*g.y-f.y*g.x > 0
    delay: f32
    if left { delay = kind_length(v.kind)*0.5 }
    c, problem = connector_candidate(w, v, exit, lateral, delay, 1, 0)
    if problem != .None && problem != .Tight_Turn { return }
    // Only explicit uncontrolled single-arm map endpoints retain the old
    // overhang abstraction. Internal/controlled terminal bodies must fit too.
    if exit < 0 && terminal_is_external(w, v.road) { return }
    hull := junction_polygon(w, w.roads[v.road].to)
    defer delete(hull)
    if problem == .None && connector_on_pavement(w, c, v.kind, hull[:]) { return }
    if exit < 0 { return Connector{}, .Terminal_Clearance }
    // Bounded local candidates, not a generic steering/graph solver.
    // First try the tangent intersection; then shorter handles and delayed
    // mouth entry. Every accepted path uses the exact rendered asphalt union.
    // Steering may need to finish before the front enters a narrow exit. The
    // straight tail carries the body until its REAR clears the junction.
    early_tail := junction_clearance_distance(w, exit)+kind_length(v.kind)-junction_mouth_depth(w, exit)
    for tail in ([]f32{0, early_tail, early_tail+kind_width(v.kind)*0.5}) {
        for extra in ([]f32{0, kind_width(v.kind)*0.5, kind_width(v.kind), kind_width(v.kind)*1.5, -delay}) {
            for factor in ([]f32{1,0.75,0.5}) {
                if tail == 0 && extra == 0 && factor == 1 { continue }
                candidate, why := connector_candidate(w, v, exit, lateral, delay+extra, factor, tail)
                if why == .None && connector_on_pavement(w, candidate, v.kind, hull[:]) { return candidate, .None }
            }
        }
    }
    // An uncontrolled approach has no zebra: a turning body may begin at its
    // entry instead of the mouth, if the same pavement test passes.
    // Controlled approaches keep the crossing straight and untouched.
    if signal_state(w, v.road) == .Uncontrolled && f.x*g.y-f.y*g.x != 0 {
        early := max(v.distance, entry_distance(w, v.road, v.kind))-turn_start_distance(w, v.road)
        for factor in ([]f32{1,0.75,0.5}) {
            candidate, why := connector_candidate(w, v, exit, lateral, early, factor, 0)
            if why == .None && connector_on_pavement(w, candidate, v.kind, hull[:]) { return candidate, .None }
        }
    }
    return Connector{}, .Tight_Turn
}

connector_candidate :: proc(w: ^World, v: Vehicle, exit: int, lateral, delay, factor, tail: f32) -> (c: Connector, problem: Motion_Problem) {
    length := road_length(w, v.road)
    if length <= 0.001 { return c, .Degenerate }
    entry := entry_distance(w, v.road, v.kind)
    _, _, _, _, crossing_ok := crossing_band(w, v.road)
    if entry < 0 || (signal_state(w, v.road) != .Uncontrolled && !crossing_ok) { return c, .Short_Approach }
    if exit < 0 && !terminal_clearance_ok(w, v.road, v.kind) { return c, .Terminal_Clearance }
    end, outgoing := length, length
    if exit >= 0 {
        outgoing = road_length(w, exit)
        if outgoing <= 0.001 { return c, .Degenerate }
        // Clear the whole incident hull/strip envelope, including wide/skew arms.
        end = junction_clearance_distance(w, exit)+kind_length(v.kind)*0.5
        if end+kind_length(v.kind)*0.5 > outgoing { return c, .Short_Exit }
        next := v
        next.road = exit
        if (successor(w, next) >= 0 || signal_state(w, exit) != .Uncontrolled) && end > entry_distance(w, exit, v.kind) { return c, .Short_Exit }
    }
    a := v
    a.motion = {}
    a.distance = max(entry, v.distance) // already committed vehicles may clear after red
    start, forward := vehicle_pose(w, a)
    b := Vehicle{road = exit, distance = end, lateral = lateral, kind = v.kind}
    if exit < 0 { b.road, b.lateral = v.road, v.lateral }
    q, g := vehicle_pose(w, b)
    q.x -= g.x*tail
    q.y -= g.y*tail
    f := forward
    cross := f.x*g.y-f.y*g.x
    turn := turn_start_distance(w, v.road)
    // Left starts at rear clearance; right starts at the mouth center. The
    // fitter may delay further; only its uncontrolled fallback starts earlier.
    turn += delay
    parallel_shift := abs(cross) <= 0.00001 && abs((q.x-start.x)*f.y-(q.y-start.y)*f.x) > 0.00001
    // No zebra exists here: an uncontrolled width/lateral convergence may
    // start at its approach, rather than waiting too late at the mouth.
    if parallel_shift && signal_state(w, v.road) == .Uncontrolled { turn = a.distance+delay }
    turn = max(a.distance, turn)
    if exit < 0 { turn = min(turn, length) }
    lead := turn-a.distance
    p := Vec2{start.x+f.x*lead, start.y+f.y*lead}
    if f.x*g.x+f.y*g.y < -0.95 { return c, .U_Turn }
    handle := vec_distance(p, q)*0.45*factor
    if abs(cross) <= 0.00001 && exit >= 0 && (q.x-p.x)*f.x+(q.y-p.y)*f.y <= handle { return c, .Tight_Turn }
    first, second := Vec2{p.x+f.x*handle, p.y+f.y*handle}, Vec2{q.x-g.x*handle, q.y-g.y*handle}
    if parallel_shift {
        along := (q.x-p.x)*f.x+(q.y-p.y)*f.y
        // A long initial tangent avoids swinging an outer-lane rear past the
        // incoming curb; the shorter final tangent aligns before the tail.
        first = Vec2{p.x+f.x*along*1.05*factor, p.y+f.y*along*1.05*factor}
        second = Vec2{q.x-g.x*along*0.05*factor, q.y-g.y*along*0.05*factor}
    }
    if abs(cross) > 0.00001 {
        delta := Vec2{q.x-p.x, q.y-p.y}
        incoming := (delta.x*g.y-delta.y*g.x)/cross
        outgoing := (f.x*delta.y-f.y*delta.x)/cross
        if incoming <= 0 || outgoing <= 0 { return c, .Tight_Turn }
        // Meeting both tangent handles at their line intersection avoids the
        // distance-scaled handle overshoot that clipped bus/car body corners.
        first = Vec2{p.x+f.x*incoming*factor, p.y+f.y*incoming*factor}
        second = Vec2{q.x-g.x*outgoing*factor, q.y-g.y*outgoing*factor}
    }
    c = Connector{active = true, node = w.roads[v.road].to, exit = exit, end = end, lateral = lateral, start = start, forward = forward, lead = lead, tail = tail, outgoing = g,
        controls = {p, first, second, q}}
    previous := p
    for i in 1..<len(c.arc) {
        point, _ := bezier_pose(c, f32(i)/64)
        c.arc[i] = c.arc[i-1]+vec_distance(previous, point)
        previous = point
    }
    c.length = lead+c.arc[64]+tail
    return c, .None
}

// Oriented body SAT, in ordinary world coordinates across ALL road IDs.
// A small margin supplements the road-local longitudinal/lateral admission gaps.
bodies_overlap :: proc(w: ^World, a, b: Vehicle, margin: f32 = 0.025) -> bool {
    p, f := vehicle_pose(w, a)
    q, g := vehicle_pose(w, b)
    axes := [4]Vec2{f, {-f.y, f.x}, g, {-g.y, g.x}}
    radius := (kind_length(a.kind)+kind_width(a.kind)+kind_length(b.kind)+kind_width(b.kind))*0.5+margin
    if (q.x-p.x)*(q.x-p.x)+(q.y-p.y)*(q.y-p.y) > radius*radius { return false }
    for axis in axes {
        separation := abs((q.x-p.x)*axis.x+(q.y-p.y)*axis.y)
        ra := kind_length(a.kind)*0.5*abs(f.x*axis.x+f.y*axis.y)+kind_width(a.kind)*0.5*abs(-f.y*axis.x+f.x*axis.y)
        rb := kind_length(b.kind)*0.5*abs(g.x*axis.x+g.y*axis.y)+kind_width(b.kind)*0.5*abs(-g.y*axis.x+g.x*axis.y)
        if separation >= ra+rb+margin { return false }
    }
    return true
}
body_clear :: proc(w: ^World, candidate: Vehicle, ignore: int = -1) -> bool {
    for other, i in w.vehicles {
        if i != ignore && bodies_overlap(w, candidate, other) { return false }
    }
    return true
}

// Exact collinearity selects straight speed targets, not crossing permission.
// Monotone projections also exclude a reversing control polygon.
straight_direction :: proc(c: Connector) -> (forward: Vec2, straight: bool) {
    start := c.start
    end := Vec2{c.controls[3].x+c.outgoing.x*c.tail, c.controls[3].y+c.outgoing.y*c.tail}
    d := Vec2{end.x-start.x, end.y-start.y}
    length := vec_distance(start, end)
    if length <= 0.001 { return }
    previous: f32 = 0
    for point in ([5]Vec2{c.controls[0], c.controls[1], c.controls[2], c.controls[3], end}) {
        q := Vec2{point.x-start.x, point.y-start.y}
        along := q.x*d.x+q.y*d.y
        if q.x*d.y-q.y*d.x != 0 || along < previous { return }
        previous = along
    }
    return Vec2{d.x/length, d.y/length}, true
}

parallel_bands_disjoint :: proc(a, af: Vec2, aw: f32, b, bf: Vec2, bw: f32) -> bool {
    if af.x*bf.y-af.y*bf.x != 0 || af.x*bf.x+af.y*bf.y <= 0 { return false }
    separation := abs(-(a.x-b.x)*af.y+(a.y-b.y)*af.x)
    return separation >= (aw+bw)*0.5+0.18
}

straight_corridors_disjoint :: proc(a: Connector, ak: Vehicle_Kind, b: Connector, bk: Vehicle_Kind) -> bool {
    af, a_straight := straight_direction(a)
    bf, b_straight := straight_direction(b)
    if !a_straight || !b_straight { return false }
    return parallel_bands_disjoint(a.start, af, kind_width(ak), b.start, bf, kind_width(bk))
}


source_problem :: proc(w: ^World, v: Vehicle) -> Motion_Problem {
    exit := successor(w, v)
    if road_length(w, v.road) <= 0.001 { return .Degenerate }
    // Terminal trips still need enough approach for their front to wait at
    // the shared line. Green is not permission to spawn beyond commitment.
    if entry_distance(w, v.road, v.kind) < 0 { return .Short_Approach }
    _, _, _, _, crossing_ok := crossing_band(w, v.road)
    if signal_state(w, v.road) != .Uncontrolled && !crossing_ok { return .Short_Approach }
    if exit < 0 {
        if !terminal_clearance_ok(w, v.road, v.kind) { return .Terminal_Clearance }
        return .None
    }
    // Source admission rejects only structural geometry. A center-exit body
    // trial is not evidence that the actual incoming/receiving lateral choices
    // will all fail. The shared node admission evaluates those actual choices.
    _, problem := connector_candidate(w, v, exit, 0, 0, 1, 0)
    if problem == .Tight_Turn { return .None }
    return problem
}

source_geometry_problem :: proc(w: ^World, v: Vehicle) -> Motion_Problem {
    problem := source_problem(w, v)
    exit := successor(w, v)
    if problem == .None && exit < 0 {
        _, problem = make_connector(w, v, exit, v.lateral)
        return problem
    }
    if problem == .None && exit >= 0 {
        candidates := connector_laterals(w, v, exit)
        defer delete(candidates)
        problem = .Short_Exit
        for lateral in candidates {
            _, problem = make_connector(w, v, exit, lateral)
            if problem == .None { return .None }
            if problem != .Tight_Turn { break }
        }
    }
    return problem
}

source_supported :: proc(w: ^World, v: Vehicle) -> bool {
    problem := source_geometry_problem(w, v)
    if problem == .None { return true }
    report_motion_issue(w, v, successor(w, v), problem)
    return false
}

connector_laterals :: proc(w: ^World, v: Vehicle, exit: int) -> [dynamic]f32 {
    width := w.roads[v.road].width
    if exit >= 0 { width = w.roads[exit].width }
    limit := max(0, width*0.5-kind_width(v.kind)*0.5)
    if width < kind_width(v.kind) { return nil }
    desired := clamp(v.preferred_lateral, -limit, limit)
    candidates := make([dynamic]f32, 0, 2*len(w.vehicles)+3)
    // Preserve an occupied world-space band across aligned links when it fits.
    // Fitting, local gaps and body checks still decide whether it is usable.
    if exit >= 0 {
        _, incoming := vehicle_pose(w, Vehicle{road = v.road})
        _, outgoing := vehicle_pose(w, Vehicle{road = exit})
        if incoming.x*outgoing.y-incoming.y*outgoing.x == 0 && incoming.x*outgoing.x+incoming.y*outgoing.y > 0 {
            aligned := v.lateral+road_offset(w, v.road)-road_offset(w, exit)
            if aligned >= -limit && aligned <= limit { append(&candidates, aligned) }
        }
    }
    append(&candidates, desired, -limit, limit, 0)
    if exit < 0 { clear(&candidates); append(&candidates, v.lateral) }
    for other in w.vehicles {
        if other.road != exit { continue }
        gap := (kind_width(v.kind)+kind_width(other.kind))*0.5+0.18001
        append(&candidates, clamp(other.lateral-gap, -limit, limit), clamp(other.lateral+gap, -limit, limit))
    }
    return candidates
}

// Predict only nearby motion, not ownership of a turn or an outgoing ray.
// An uncommitted vehicle never gets forecast through a signal/next junction.
forecast_vehicle :: proc(w: ^World, v: Vehicle, distance: f32) -> Vehicle {
    trial := v
    left := distance
    if trial.motion.active {
        amount := min(left, trial.motion.length-trial.motion.progress)
        trial.motion.progress += amount
        left -= amount
        if trial.motion.progress < trial.motion.length || trial.motion.exit < 0 { return trial }
        trial.road, trial.distance, trial.lateral = trial.motion.exit, trial.motion.end, trial.motion.lateral
        trial.motion = {}
    }
    trial.distance += min(left, max(0, entry_distance(w, trial.road, trial.kind)-trial.distance))
    if trial.lane.active { trial.lateral, _ = lane_offset(trial) }
    return trial
}

// Synthetic 1.5s rolling gap acceptance. Older entrants have tie priority at
// an imminent conflict, not a claim on their remaining route. Physical bodies
// always block regardless of priority. O(n * horizon / 0.05s), capped at 120;
// use a spatial neighbor index before increasing the population.
// This constant-speed prediction is not a collision/deadlock proof.
local_gap_clear :: proc(w: ^World, candidate: Vehicle, ignore: int, speed: f32) -> bool {
    horizon :: f32(1.5)
    for other, j in w.vehicles {
        if j == ignore { continue }
        // Same-link followers already yield through longitudinal following.
        // Extrapolating them through the departing body would veto escape.
        if other.road == candidate.road && !other.motion.active && !other.lane.active &&
            other.distance < candidate.distance && abs(other.lateral-candidate.lateral) < (kind_width(other.kind)+kind_width(candidate.kind))*0.5+0.18 { continue }
        if other.motion.active && candidate.motion.sequence != 0 &&
            other.motion.sequence > candidate.motion.sequence { continue }
        // Observe stopped traffic as stopped. Inventing forward creep for
        // a blocked rider can prevent the crossing rider from clearing it.
        other_speed := other.speed
        p, _ := vehicle_pose(w, candidate)
        q, _ := vehicle_pose(w, other)
        reach := (speed+other_speed)*horizon+kind_length(candidate.kind)+kind_length(other.kind)
        if vec_distance(p, q) > reach { continue }
        for k in 1..=30 {
            time := f32(k)*0.05
            a := forecast_vehicle(w, candidate, speed*time)
            b := forecast_vehicle(w, other, other_speed*time)
            if bodies_overlap(w, a, b) { return false }
        }
    }
    return true
}

acquire_connector :: proc(w: ^World, i, exit: int) -> bool {
    v := w.vehicles[i]
    candidates := connector_laterals(w, v, exit)
    defer delete(candidates)
    if len(candidates) == 0 { report_motion_issue(w, v, exit, .Short_Exit); return false }
    unfit, geometry_found := false, false
    for lateral in candidates {
        c, problem := make_connector(w, v, exit, lateral)
        if problem == .Tight_Turn { unfit = true; continue }
        if problem != .None { report_motion_issue(w, v, exit, problem); return false }
        geometry_found = true
        c.sequence = w.entry_sequence+1
        trial := v
        trial.motion = c
        // Keep the first geometrically usable band. A temporary conflict is
        // a reason to yield, not to cut across neighboring riders' bands.
        if !body_clear(w, trial, i) || !local_gap_clear(w, trial, i, v.speed) { return false }
        w.entry_sequence += 1
        w.vehicles[i].motion = c
        return true
    }
    if unfit && !geometry_found { report_motion_issue(w, v, exit, .Tight_Turn) }
    return false
}

// Advance along actual simulation segments, carrying the budget across both
// connector boundaries. <= 0.10m spatial checks prevent tunnelling at coarse dt;
// this sampled body model is not a certification of arbitrary swept geometry.
move_vehicle :: proc(w: ^World, i: int, budget: f32) -> (travelled: f32, finished: bool) {
    left := budget
    for left > 0.000001 {
        v := w.vehicles[i]
        if !v.motion.active {
            exit := successor(w, v)
            if exit < 0 && v.distance >= road_length(w, v.road)-0.000001 {
                _, problem := make_connector(w, v, exit, v.lateral)
                if problem != .None { report_motion_issue(w, v, exit, problem); break }
                return travelled, true
            }
            boundary := max(v.distance, max(0, entry_distance(w, v.road, v.kind)))
            state := signal_state(w, v.road)
            stop := entry_distance(w, v.road, v.kind)
            if (state == .Red || state == .Amber) && v.distance <= stop+0.0001 {
                boundary = min(boundary, max(0, stop))
            }
            if v.distance >= boundary-0.000001 {
                if (state == .Red || state == .Amber) && v.distance <= stop+0.0001 { break }
                if !acquire_connector(w, i, exit) { break }
                continue
            }
            if v.lane.active && !lane_rear_safe(w, i, v, false) { break }
            amount := min(left, min(0.1, boundary-v.distance))
            trial := v
            trial.distance += amount
            if trial.lane.active {
                trial.lateral, _ = lane_offset(trial)
                if !local_gap_clear(w, trial, i, v.speed) { break }
                if trial.distance >= trial.lane.start+LANE_CHANGE_LENGTH { trial.lane = {} }
            }
            if !body_clear(w, trial, i) { break }
            w.vehicles[i] = trial
            left -= amount
            travelled += amount
        } else {
            amount := min(left, min(0.1, v.motion.length-v.motion.progress))
            trial := v
            trial.motion.progress += amount
            if !body_clear(w, trial, i) || !local_gap_clear(w, trial, i, v.speed) { break }
            if trial.motion.progress >= trial.motion.length-0.000001 {
                if trial.motion.exit < 0 { return travelled+amount, true }
                trial.road, trial.distance, trial.lateral = trial.motion.exit, trial.motion.end, trial.motion.lateral
                trial.motion = {}
            }
            w.vehicles[i] = trial
            left -= amount
            travelled += amount
        }
    }
    return
}
