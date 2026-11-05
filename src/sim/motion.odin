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
    // A local bypass is two tangent-matched cubics; continuation starts at
    // controls[3]. Ordinary fitted connectors remain a single cubic.
    bypass: bool,
    continuation: [3]Vec2,
    // Adaptive guides are short forward steering actions, not a return to the
    // old cubic. Cached action-boundary poses keep forecasts O(1).
    steering_count: int,
    steering: [32]f32,
    steering_poses: [33][2]Vec2,
    partial: bool,
    local: bool, // Adaptive same-link maneuver; never advances the route edge.
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
    if c.bypass {
        if t > 0.5 { return cubic_pose({c.controls[3], c.continuation[0], c.continuation[1], c.continuation[2]}, 2*t-1) }
        return cubic_pose(c.controls, 2*t)
    }
    return cubic_pose(c.controls, t)
}

cubic_pose :: proc(controls: [4]Vec2, t: f32) -> (p, f: Vec2) {
    s := 1-t
    a, b, d, e := controls[0], controls[1], controls[2], controls[3]
    p = {s*s*s*a.x+3*s*s*t*b.x+3*s*t*t*d.x+t*t*t*e.x,
         s*s*s*a.y+3*s*s*t*b.y+3*s*t*t*d.y+t*t*t*e.y}
    f = {3*s*s*(b.x-a.x)+6*s*t*(d.x-b.x)+3*t*t*(e.x-d.x),
         3*s*s*(b.y-a.y)+6*s*t*(d.y-b.y)+3*t*t*(e.y-d.y)}
    n := math.sqrt(f.x*f.x+f.y*f.y)
    if n > 0 { f.x /= n; f.y /= n }
    return
}
STEERING_STEP :: f32(0.25)
STEERING_SPEED :: f32(2)
STEERING_YAW_RATE :: f32(1)

curvature_pose :: proc(p, f: Vec2, curvature, distance: f32) -> (position, forward: Vec2) {
    if curvature == 0 { return Vec2{p.x+f.x*distance, p.y+f.y*distance}, f }
    s, c := math.sin(curvature*distance), math.cos(curvature*distance)
    return Vec2{p.x+(f.x*s+f.y*(c-1))/curvature, p.y+(f.y*s-f.x*(c-1))/curvature},
        Vec2{f.x*c-f.y*s, f.y*c+f.x*s}
}

connector_pose :: proc(c: Connector, distance: f32) -> (p, f: Vec2) {
    d := clamp(distance, 0, c.length)
    if c.steering_count > 0 {
        if d >= c.length { return c.steering_poses[c.steering_count][0], c.steering_poses[c.steering_count][1] }
        index := min(int(d/STEERING_STEP), c.steering_count-1)
        pose := c.steering_poses[index]
        return curvature_pose(pose[0], pose[1], c.steering[index], d-f32(index)*STEERING_STEP)
    }
    if d <= c.lead { return Vec2{c.start.x+c.forward.x*d, c.start.y+c.forward.y*d}, c.forward }
    d -= c.lead
    if d >= c.arc[64] {
        tail := d-c.arc[64]
        q := c.controls[3]
        if c.bypass { q = c.continuation[2] }
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
    if len(w.spawns) == 0 { return choose_next_road(w, v) }
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

uturn_kind :: proc(kind: Vehicle_Kind) -> bool {
    return kind == .Motorcycle || kind == .Bicycle || kind == .Tricycle
}

heading_reverses :: proc(incoming, outgoing: Vec2) -> bool {
    return incoming.x*outgoing.x+incoming.y*outgoing.y < -0.95
}

connector_is_left :: proc(c: Connector) -> bool {
    cross := c.forward.x*c.outgoing.y-c.forward.y*c.outgoing.x
    along := c.forward.x*c.outgoing.x+c.forward.y*c.outgoing.y
    return cross > 0.2 && along > -0.5
}

// Two-wheelers may reverse heading with a left-bulging cubic that stays on the
// incident asphalt. Cars, buses and trucks remain an explicit U_Turn hold.
fit_uturn_connector :: proc(w: ^World, v: Vehicle, exit: int, lateral: f32) -> (c: Connector, problem: Motion_Problem) {
    if exit < 0 || !uturn_kind(v.kind) { return c, .U_Turn }
    length := road_length(w, v.road)
    if length <= 0.001 { return c, .Degenerate }
    entry := entry_distance(w, v.road, v.kind)
    if entry < 0 { return c, .Short_Approach }
    outgoing := road_length(w, exit)
    if outgoing <= 0.001 { return c, .Degenerate }
    end := junction_clearance_distance(w, exit)+kind_length(v.kind)*0.5
    if end+kind_length(v.kind)*0.5 > outgoing { return c, .Short_Exit }
    next := v
    next.road = exit
    if (successor(w, next) >= 0 || signal_state(w, exit) != .Uncontrolled) && end > entry_distance(w, exit, v.kind) {
        return c, .Short_Exit
    }
    a := v
    a.motion = {}
    a.distance = max(entry, v.distance)
    start, forward := vehicle_pose(w, a)
    q, g := vehicle_pose(w, Vehicle{road = exit, distance = end, lateral = lateral, kind = v.kind})
    hull := junction_polygon(w, w.roads[v.road].to)
    defer delete(hull)
    // A curbside rear needs to enter the wider junction before rotating. On a
    // controlled approach this also preserves the protected straight crossing.
    mouth_lead := max(0, turn_start_distance(w, v.road)+kind_length(v.kind)*0.5-a.distance)
    for lead in ([2]f32{0, mouth_lead}) {
        if lead == 0 && mouth_lead > 0 && signal_state(w, v.road) != .Uncontrolled { continue }
        p := Vec2{start.x+forward.x*lead, start.y+forward.y*lead}
        for handle in ([]f32{4, 5, 6, 3.2, 7, 8, 2.6}) {
            first := Vec2{p.x+forward.x*handle, p.y+forward.y*handle}
            second := Vec2{q.x-g.x*handle, q.y-g.y*handle}
            candidate := Connector{
                active = true, node = w.roads[v.road].to, exit = exit, end = end, lateral = lateral,
                start = start, forward = forward, lead = lead, outgoing = g, controls = {p, first, second, q},
            }
            previous := p
            for i in 1..<len(candidate.arc) {
                point, _ := bezier_pose(candidate, f32(i)/64)
                candidate.arc[i] = candidate.arc[i-1]+vec_distance(previous, point)
                previous = point
            }
            candidate.length = lead+candidate.arc[64]
            if candidate.length <= 0.5 { continue }
            if connector_on_pavement(w, candidate, v.kind, hull[:]) { return candidate, .None }
        }
    }
    return c, .U_Turn
}

fit_connector :: proc(w: ^World, v: Vehicle, exit: int, lateral: f32) -> (c: Connector, problem: Motion_Problem) {
    _, f := vehicle_pose(w, v)
    g := f
    if exit >= 0 { _, g = vehicle_pose(w, Vehicle{road = exit}) }
    if heading_reverses(f, g) {
        if uturn_kind(v.kind) {
            c, problem = fit_uturn_connector(w, v, exit, lateral)
            if problem == .None { return }
        }
        return {}, .U_Turn
    }
    left := f.x*g.y-f.y*g.x > 0
    delay: f32
    if left { delay = kind_length(v.kind)*0.5 }
    // On an uncontrolled left turn, start the diagonal approach at the
    // entry line when the whole body fits; rolling gap checks can then stop
    // the driver partway through the turn for opposing traffic. Wider handles
    // keep a car in the box instead of snapping 90° at the mouth.
    if left && signal_state(w, v.road) == .Uncontrolled {
        early := max(v.distance, entry_distance(w, v.road, v.kind))-turn_start_distance(w, v.road)
        hull := junction_polygon(w, w.roads[v.road].to)
        for factor in ([]f32{1, 1.2, 1.4, 0.85}) {
            candidate, why := connector_candidate(w, v, exit, lateral, early, factor, 0)
            if why == .None && connector_on_pavement(w, candidate, v.kind, hull[:]) {
                delete(hull)
                return candidate, .None
            }
        }
        delete(hull)
    }
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
    return body_poses_overlap(p, f, a.kind, q, g, b.kind, margin)
}

body_poses_overlap :: proc(p, f: Vec2, a: Vehicle_Kind, q, g: Vec2, b: Vehicle_Kind, margin: f32 = 0.025) -> bool {
    axes := [4]Vec2{f, {-f.y, f.x}, g, {-g.y, g.x}}
    radius := (kind_length(a)+kind_width(a)+kind_length(b)+kind_width(b))*0.5+margin
    if (q.x-p.x)*(q.x-p.x)+(q.y-p.y)*(q.y-p.y) > radius*radius { return false }
    for axis in axes {
        separation := abs((q.x-p.x)*axis.x+(q.y-p.y)*axis.y)
        ra := kind_length(a)*0.5*abs(f.x*axis.x+f.y*axis.y)+kind_width(a)*0.5*abs(-f.y*axis.x+f.x*axis.y)
        rb := kind_length(b)*0.5*abs(g.x*axis.x+g.y*axis.y)+kind_width(b)*0.5*abs(-g.y*axis.x+g.x*axis.y)
        if separation >= ra+rb+margin { return false }
    }
    return true
}
body_clear :: proc(w: ^World, candidate: Vehicle, ignore: int = -1, blocker: ^u64 = nil) -> bool {
    if blocker != nil { blocker^ = 0 }
    p, f := vehicle_pose(w, candidate)
    for other, i in w.vehicles {
        if i == ignore { continue }
        q, g := vehicle_pose(w, other)
        if body_poses_overlap(p, f, candidate.kind, q, g, other.kind) { if blocker != nil { blocker^ = other.id }; return false }
    }
    return true
}

// Exact collinearity selects straight speed targets, not crossing permission.
// Monotone projections also exclude a reversing control polygon.
straight_direction :: proc(c: Connector) -> (forward: Vec2, straight: bool) {
    if c.bypass { return {}, false }
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
    if problem == .U_Turn && uturn_kind(v.kind) { return .None }
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
            if problem != .Tight_Turn && problem != .U_Turn { break }
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

receiving_desired :: proc(w: ^World, v: Vehicle, exit: int, limit: f32) -> f32 {
    desired := clamp(v.preferred_lateral, -limit, limit)
    if exit < 0 || v.driver.turn_bias == 0 || (v.kind != .Motorcycle && v.kind != .Bicycle) { return desired }
    _, incoming := vehicle_pose(w, Vehicle{road = v.road})
    _, outgoing := vehicle_pose(w, Vehicle{road = exit})
    cross := incoming.x*outgoing.y-incoming.y*outgoing.x
    if abs(cross) <= 0.00001 { return desired }
    inside: f32 = 1
    if cross < 0 { inside = -1 }
    return clamp(desired+v.driver.turn_bias*inside*limit, -limit, limit)
}

connector_laterals :: proc(w: ^World, v: Vehicle, exit: int) -> [dynamic]f32 {
    width := w.roads[v.road].width
    if exit >= 0 { width = w.roads[exit].width }
    limit := max(0, width*0.5-kind_width(v.kind)*0.5)
    if width < kind_width(v.kind) { return nil }
    desired := receiving_desired(w, v, exit, limit)
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
    start := 0
    if exit >= 0 && len(candidates) > 1 && candidates[0] != desired { start = 1 }
    for i in start..<len(candidates) {
        best := i
        for j in i+1..<len(candidates) {
            if abs(candidates[j]-desired) < abs(candidates[best]-desired) { best = j }
        }
        candidates[i], candidates[best] = candidates[best], candidates[i]
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
        if trial.motion.progress < trial.motion.length || trial.motion.exit < 0 || trial.motion.partial { return trial }
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
local_gap_clear :: proc(w: ^World, candidate: Vehicle, ignore: int, speed: f32, horizon := f32(1.5), blocker: ^u64 = nil) -> bool {
    if blocker != nil { blocker^ = 0 }
    p, _ := vehicle_pose(w, candidate)
    samples := int(horizon/0.05+0.001)
    if samples < 1 { samples = 1 }
    if samples > 30 { samples = 30 }
    // Cache only within this read-only query: sequential arbitration can change
    // poses between calls. Keep every 50ms prediction and exact body SAT check.
    poses: [30][2]Vec2
    predicted := 0
    for other, j in w.vehicles {
        if j == ignore { continue }
        // Same-link followers already yield through longitudinal following.
        // Extrapolating them through the departing body would veto escape.
        if other.road == candidate.road && !other.motion.active && !other.lane.active &&
            other.distance < candidate.distance && abs(other.lateral-candidate.lateral) < (kind_width(other.kind)+kind_width(candidate.kind))*0.5+0.18 { continue }
        if other.motion.active && !candidate.motion.local && !other.motion.local && candidate.motion.sequence != 0 &&
            other.motion.sequence > candidate.motion.sequence { continue }
        // Observe stopped traffic as stopped. Inventing forward creep for
        // a blocked rider can prevent the crossing rider from clearing it.
        other_speed := other.speed
        q, g := vehicle_pose(w, other)
        reach := (speed+other_speed)*horizon+kind_length(candidate.kind)+kind_length(other.kind)
        if vec_distance(p, q) > reach { continue }
        if other_speed == 0 {
            b := forecast_vehicle(w, other, 0)
            q, g = vehicle_pose(w, b)
        }
        for k in 1..=samples {
            time := f32(k)*0.05
            index := k-1 if speed != 0 else 0
            if index >= predicted {
                a := forecast_vehicle(w, candidate, speed*time)
                poses[index][0], poses[index][1] = vehicle_pose(w, a)
                predicted = index+1
            }
            if other_speed != 0 {
                b := forecast_vehicle(w, other, other_speed*time)
                q, g = vehicle_pose(w, b)
            }
            if body_poses_overlap(poses[index][0], poses[index][1], candidate.kind, q, g, other.kind) { if blocker != nil { blocker^ = other.id }; return false }
            if speed == 0 && other_speed == 0 { break } // All samples are identical.
        }
    }
    return true
}

acquire_connector :: proc(w: ^World, i, exit: int, feedback: ^Driver_Feedback = nil) -> bool {
    if feedback != nil { feedback^ = {attempted = true} }
    v := w.vehicles[i]
    candidates := connector_laterals(w, v, exit)
    defer delete(candidates)
    if len(candidates) == 0 { report_motion_issue(w, v, exit, .Short_Exit); set_driver_feedback(feedback, .Geometry, 0, .Short_Exit); return false }
    unfit, geometry_found := false, false
    for lateral in candidates {
        c, problem := make_connector(w, v, exit, lateral)
        if problem == .Tight_Turn || problem == .U_Turn { unfit = true; continue }
        if problem != .None { report_motion_issue(w, v, exit, problem); set_driver_feedback(feedback, .Geometry, 0, problem); return false }
        geometry_found = true
        c.sequence = w.entry_sequence+1
        trial := v
        trial.motion = c
        // Prefer the nearest receiving band, but if its trajectory is occupied,
        // try the next fitted band rather than waiting behind a face-to-face
        // conflict. Each alternative still passes the same body and time-gap checks.
        blocker: u64
        if !body_clear(w, trial, i, &blocker) {
            set_driver_feedback(feedback, .Body, blocker)
            continue
        }
        if !local_gap_clear(w, trial, i, v.speed, gap_horizon(v.driver), &blocker) {
            set_driver_feedback(feedback, .Predicted_Conflict, blocker)
            continue
        }
        if feedback != nil { feedback^ = {attempted = true, accepted = true} }
        w.entry_sequence += 1
        w.vehicles[i].motion = c
        return true
    }
    if unfit && !geometry_found {
        _, f := vehicle_pose(w, v)
        g := f
        if exit >= 0 { _, g = vehicle_pose(w, Vehicle{road = exit}) }
        problem := heading_reverses(f, g) ? Motion_Problem.U_Turn : .Tight_Turn
        report_motion_issue(w, v, exit, problem)
        set_driver_feedback(feedback, .Geometry, 0, problem)
    }
    return false
}

// Advance along actual simulation segments, carrying the budget across both
// connector boundaries. <= 0.10m spatial checks prevent tunnelling at coarse dt;
// this sampled body model is not a certification of arbitrary swept geometry.
move_vehicle :: proc(w: ^World, i: int, budget: f32, feedback: ^Driver_Feedback = nil) -> (travelled: f32, finished: bool) {
    if feedback != nil { feedback^ = {attempted = budget > 0} }
    defer if feedback != nil {
        feedback.moved = travelled
        feedback.finished = finished
        feedback.accepted = travelled > 0 || finished
        feedback.partial = travelled > 0 && !finished && travelled+0.000001 < budget
    }
    blocker: u64
    hull: [dynamic]Vec2
    if w.vehicles[i].motion.steering_count > 0 && !w.vehicles[i].motion.local { hull = junction_polygon(w, w.vehicles[i].motion.node) }
    defer delete(hull)
    left := budget
    for left > 0.000001 {
        v := w.vehicles[i]
        if !v.motion.active {
            exit := successor(w, v)
            if exit < 0 && v.distance >= road_length(w, v.road)-0.000001 {
                _, problem := make_connector(w, v, exit, v.lateral)
                if problem != .None { report_motion_issue(w, v, exit, problem); set_driver_feedback(feedback, .Geometry, 0, problem); break }
                return travelled, true
            }
            boundary := max(v.distance, max(0, entry_distance(w, v.road, v.kind)))
            state := signal_state(w, v.road)
            stop := entry_distance(w, v.road, v.kind)
            if (state == .Red || state == .Amber) && v.distance <= stop+0.0001 {
                boundary = min(boundary, max(0, stop))
            }
            if v.distance >= boundary-0.000001 {
                if (state == .Red || state == .Amber) && v.distance <= stop+0.0001 { set_driver_feedback(feedback, .Signal); break }
                admission: Driver_Feedback
                if !acquire_connector(w, i, exit, &admission) {
                    set_driver_feedback(feedback, admission.reason, admission.blocker, admission.problem)
                    break
                }
                continue
            }
            if v.lane.active && !lane_rear_safe(w, i, v, false, &blocker) { set_driver_feedback(feedback, .Rear_Gap, blocker); break }
            amount := min(left, min(0.1, boundary-v.distance))
            trial := v
            trial.distance += amount
            if trial.lane.active {
                trial.lateral, _ = lane_offset(trial)
                if !local_gap_clear(w, trial, i, v.speed, gap_horizon(v.driver), &blocker) { set_driver_feedback(feedback, .Predicted_Conflict, blocker); break }
                if trial.distance >= trial.lane.start+lane_change_length(trial) { trial.lane = {} }
            }
            if !body_clear(w, trial, i, &blocker) { set_driver_feedback(feedback, .Body, blocker); break }
            w.vehicles[i] = trial
            left -= amount
            travelled += amount
        } else {
            amount := min(left, min(0.1, v.motion.length-v.motion.progress))
            trial := v
            trial.motion.progress += amount
            if trial.motion.steering_count > 0 {
                p, f := vehicle_pose(w, trial)
                permitted := -1
                if trial.motion.local { permitted = trial.road }
                if !body_on_junction_pavement(w, trial.motion.node, p, f, trial.kind, hull[:], permitted, permitted) { set_driver_feedback(feedback, .Pavement); break }
            }
            if !body_clear(w, trial, i, &blocker) { set_driver_feedback(feedback, .Body, blocker); break }
            if !local_gap_clear(w, trial, i, v.speed, gap_horizon(v.driver), &blocker) { set_driver_feedback(feedback, .Predicted_Conflict, blocker); break }
            if trial.motion.progress >= trial.motion.length-0.000001 {
                if trial.motion.partial {
                    w.vehicles[i] = trial
                    travelled += amount
                    set_driver_feedback(feedback, .Partial_End)
                    break
                }
                if trial.motion.exit < 0 { return travelled+amount, true }
                trial.road, trial.distance, trial.lateral = trial.motion.exit, trial.motion.end, trial.motion.lateral
                trial.motion = {}
                if !body_clear(w, trial, i, &blocker) { set_driver_feedback(feedback, .Body, blocker); break }
            }
            w.vehicles[i] = trial
            left -= amount
            travelled += amount
        }
    }
    return
}
