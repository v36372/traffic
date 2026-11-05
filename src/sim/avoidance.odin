package sim

import "core:math"
import pq "core:container/priority_queue"

Steering_Node :: struct {
    p, f: Vec2,
    parent, depth: int,
    curvature, cost, remaining: f32,
}
Steering_Entry :: struct { index: int, priority: f32 }
Steering_Obstacle :: struct { p, f: Vec2, kind: Vehicle_Kind }

steering_less :: proc(a, b: Steering_Entry) -> bool {
    return a.priority < b.priority || (a.priority == b.priority && a.index < b.index)
}

// Search poses, not offsets on a rejected spline. Each action is a forward arc;
// no action rotates a stationary body, reverses, changes the exit or entry order.
// The lattice is bounded to 2048 expansions / 8m / the 120-body population cap.
// For larger junctions it executes a safe prefix and replans; raise the search
// budget or use a spatial index before raising the population/accuracy ceiling.
try_local_bypass :: proc(w: ^World, i: int, feedback: ^Driver_Feedback = nil) -> bool {
    if feedback != nil { feedback^ = {attempted = true, reason = .No_Plan} }
    blocker: u64
    v := w.vehicles[i]
    c := v.motion
    if !c.active {
        if v.speed >= 0.5 { return false }
        maximum := entry_distance(w, v.road, v.kind)
        state := signal_state(w, v.road)
        radius := 0.5*math.sqrt(kind_length(v.kind)*kind_length(v.kind)+kind_width(v.kind)*kind_width(v.kind))
        if (state == .Red || state == .Amber) && v.distance+pass_length(v)+radius >= maximum { set_driver_feedback(feedback, .Signal); return false }
        if maximum-v.distance <= 2*kind_length(v.kind)+STEERING_STEP { set_driver_feedback(feedback, .Route_Boundary); return false }
        p, f := vehicle_pose(w, v)
        c = Connector{active = true, local = true, node = w.roads[v.road].from, exit = v.road,
            end = v.distance+2*kind_length(v.kind), length = maximum-v.distance,
            start = p, forward = f, outgoing = f}
    }
    nearing_end := v.motion.active && (c.partial || v.speed < 0.5) && c.length-c.progress <= max(0.5, v.speed*v.speed/8+v.speed*0.05)
    if c.exit < 0 { set_driver_feedback(feedback, .Route_Boundary); return false }
    if c.progress < c.lead || (v.speed >= 0.5 && !nearing_end) { return false }
    // A real zero-speed execution failure is enough reason to reconsider.
    // A coarse future-body probe can miss the tiny rotation that blocked the
    // last tick, or the controller/prediction rather than a static body.
    blocked := nearing_end || v.driver.motion_blocked
    if !blocked {
        for distance := f32(0.1); distance <= min(c.length-c.progress, max(3, kind_length(v.kind))); distance += 0.1 {
            trial := forecast_vehicle(w, v, distance)
            for other, j in w.vehicles {
                if i != j && other.speed == 0 && bodies_overlap(w, trial, other) { blocked = true; break }
            }
            if blocked { break }
        }
    }
    if !blocked { return false }
    start, forward := vehicle_pose(w, v)
    origin, axis := vehicle_pose(w, Vehicle{road = c.exit})
    _, incoming_axis := vehicle_pose(w, Vehicle{road = v.road})
    // Handoff needs the entire aligned body in its receiving strip, not arrival
    // at the obsolete cubic's rear-clearance marker farther down that strip.
    // World-space movement checks still include every body in the old junction.
    minimum := kind_length(v.kind)*0.5+0.025
    if c.local { minimum = c.end }
    maximum := min(entry_distance(w, c.exit, v.kind), road_length(w, c.exit)-kind_length(v.kind)*0.5)
    if maximum < minimum { set_driver_feedback(feedback, .Route_Boundary); return false }
    along := (start.x-origin.x)*axis.x+(start.y-origin.y)*axis.y
    _, already_docked := steering_goal(start, forward, origin, axis, minimum, maximum, w.roads[c.exit].width, v.kind)
    if already_docked {
        trial := v
        trial.road, trial.distance = c.exit, along
        trial.lateral = -(start.x-origin.x)*axis.y+(start.y-origin.y)*axis.x
        trial.motion, trial.lane = {}, {}
        if !body_clear(w, trial, i, &blocker) { set_driver_feedback(feedback, .Body, blocker); return false }
        if feedback != nil { feedback^ = {attempted = true, accepted = true, finished = true} }
        w.vehicles[i] = trial
        return true
    }
    hull: [dynamic]Vec2
    if !c.local { hull = junction_polygon(w, c.node) }
    defer delete(hull)
    obstacles: [dynamic]Steering_Obstacle
    defer delete(obstacles)
    for other, j in w.vehicles {
        if i == j { continue }
        p, f := vehicle_pose(w, other)
        reach := f32(len(c.steering))*STEERING_STEP+(kind_length(v.kind)+kind_width(v.kind)+kind_length(other.kind)+kind_width(other.kind))*0.5+0.025
        if vec_distance(start, p) <= reach { append(&obstacles, Steering_Obstacle{p, f, other.kind}) }
    }
    nodes := make([dynamic]Steering_Node, 0, 256)
    defer delete(nodes)
    open: pq.Priority_Queue(Steering_Entry)
    pq.init(&open, steering_less, pq.default_swap_proc(Steering_Entry))
    defer pq.destroy(&open)
    visited := make(map[[3]int]f32)
    defer delete(visited)
    initial, _ := steering_goal(start, forward, origin, axis, minimum, maximum, w.roads[c.exit].width, v.kind)
    append(&nodes, Steering_Node{p = start, f = forward, parent = -1, remaining = initial})
    pq.push(&open, Steering_Entry{0, initial})
    best, chosen := -1, -1
    best_score := f32(1e30)
    // Retain the previous guide validator's <=acos(.98) heading change per
    // 5cm sample. This is synthetic steering, not a calibrated wheelbase model.
    curvature_limit := math.acos(f32(0.98))/0.05
    expanded := 0
    for pq.len(open) > 0 && expanded < 2048 && len(nodes) < 8192 {
        index := pq.pop(&open).index
        node := nodes[index]
        expanded += 1
        // Refine an action before expanding it, rather than trusting only its
        // coarse lattice endpoints. A corner can graze a body/curb between
        // those samples and leave execution unable to take its next tiny tick.
        if node.parent >= 0 {
            parent := nodes[node.parent]
            if !steering_action_clear(w, c.node, v.road, c.exit, incoming_axis, v.kind, parent.p, parent.f, node.curvature, hull[:], obstacles[:], 50, origin, axis, maximum) { continue }
        }
        _, goal := steering_goal(node.p, node.f, origin, axis, minimum, maximum, w.roads[c.exit].width, v.kind)
        if node.depth > 0 && goal {
            trial := v
            trial.motion = steering_connector(c, nodes[:], index, origin, axis, true)
            trial.lane = {}
            if local_gap_clear(w, trial, i, max(0.5, min(1, v.speed)), gap_horizon(v.driver)) { chosen = index; break }
        }
        // A prefix must make useful progress, not just creep into a standstill
        // gap. An eventual complete escape is not assumed from local free space.
        score := node.remaining*1.5+node.cost
        if !goal && node.depth >= 3 && node.remaining <= initial-0.5 &&
            (score < best_score-0.001 || (abs(score-best_score) <= 0.001 && (best < 0 || node.depth > nodes[best].depth))) {
            best, best_score = index, score
        }
        if node.depth >= len(c.steering) { continue }
        align := math.atan2(node.f.x*axis.y-node.f.y*axis.x, node.f.x*axis.x+node.f.y*axis.y)/STEERING_STEP
        for curvature in ([12]f32{0, clamp(align, -curvature_limit, curvature_limit),
            -curvature_limit*0.0625, curvature_limit*0.0625, -curvature_limit*0.125, curvature_limit*0.125,
            -curvature_limit*0.25, curvature_limit*0.25, -curvature_limit*0.5, curvature_limit*0.5, -curvature_limit, curvature_limit}) {
            p, f := curvature_pose(node.p, node.f, curvature, STEERING_STEP)
            along := (p.x-origin.x)*axis.x+(p.y-origin.y)*axis.y
            if along > maximum { continue }
            lateral := -(p.x-origin.x)*axis.y+(p.y-origin.y)*axis.x
            outside_exit := max(0, abs(lateral)-(w.roads[c.exit].width-kind_width(v.kind))*0.5)
            // Compare actual manoeuvring time: a tight arc is traversed slowly
            // under the yaw cap, not an artificially cheap shortcut.
            cost := node.cost+STEERING_STEP*(max(1, STEERING_SPEED*abs(curvature)/STEERING_YAW_RATE)+outside_exit/w.roads[c.exit].width)
            // Prefer room to react over a path grazing a frozen neighbour.
            // This is only a cost: the unchanged SAT margin still admits a
            // narrow squeeze when it is the available safe corridor.
            clearance := f32(0.5)
            for other in obstacles {
                if vec_distance(p, other.p) > (kind_length(v.kind)+kind_width(v.kind)+kind_length(other.kind)+kind_width(other.kind))*0.5+clearance { continue }
                separation := f32(-1e30)
                for a in ([4]Vec2{f, {-f.y,f.x}, other.f, {-other.f.y,other.f.x}}) {
                    own := kind_length(v.kind)*0.5*abs(f.x*a.x+f.y*a.y)+kind_width(v.kind)*0.5*abs(-f.y*a.x+f.x*a.y)
                    peer := kind_length(other.kind)*0.5*abs(other.f.x*a.x+other.f.y*a.y)+kind_width(other.kind)*0.5*abs(-other.f.y*a.x+other.f.x*a.y)
                    separation = max(separation, abs((p.x-other.p.x)*a.x+(p.y-other.p.y)*a.y)-own-peer)
                }
                clearance = min(clearance, separation)
            }
            cost += STEERING_STEP*max(0, 1-clearance/0.5)
            relative := Vec2{p.x-start.x, p.y-start.y}
            angle := math.atan2(forward.x*f.y-forward.y*f.x, forward.x*f.x+forward.y*f.y)
            key := [3]int{int(math.round((relative.x*forward.x+relative.y*forward.y)/0.1)),
                int(math.round((-relative.x*forward.y+relative.y*forward.x)/0.1)), int(math.round(angle*max(1, kind_length(v.kind))/0.1))}
            if old, seen := visited[key]; seen && old <= cost { continue }
            samples := 50 if node.depth == 0 else 5
            if !steering_action_clear(w, c.node, v.road, c.exit, incoming_axis, v.kind, node.p, node.f, curvature, hull[:], obstacles[:], samples, origin, axis, maximum) { continue }
            remaining, _ := steering_goal(p, f, origin, axis, minimum, maximum, w.roads[c.exit].width, v.kind)
            if len(nodes) >= 8192 { break }
            visited[key] = cost
            append(&nodes, Steering_Node{p, f, index, node.depth+1, curvature, cost, remaining})
            pq.push(&open, Steering_Entry{len(nodes)-1, cost+remaining*1.5})
        }
    }
    if chosen < 0 { chosen = best }
    if chosen < 0 {
        set_driver_feedback(feedback, (expanded >= 2048 || len(nodes) >= 8192) ? .Search_Limit : .No_Plan)
        return false
    }
    end := nodes[chosen]
    _, complete := steering_goal(end.p, end.f, origin, axis, minimum, maximum, w.roads[c.exit].width, v.kind)
    candidate := steering_connector(c, nodes[:], chosen, origin, axis, complete)
    trial := v
    trial.motion = candidate
    trial.lane = {}
    if !local_gap_clear(w, trial, i, max(0.5, min(1, v.speed)), gap_horizon(v.driver), &blocker) { set_driver_feedback(feedback, .Predicted_Conflict, blocker); return false }
    if feedback != nil { feedback^ = {attempted = true, accepted = true, partial = !complete} }
    w.vehicles[i].motion = candidate
    w.vehicles[i].lane = {}
    return true
}

steering_connector :: proc(c: Connector, nodes: []Steering_Node, chosen: int, origin, axis: Vec2, complete: bool) -> Connector {
    end := nodes[chosen]
    candidate := c
    candidate.start, candidate.forward, candidate.outgoing = nodes[0].p, nodes[0].f, axis
    candidate.progress, candidate.lead, candidate.tail = 0, 0, 0
    candidate.bypass, candidate.partial = true, !complete
    candidate.controls, candidate.continuation = {}, {}
    candidate.steering_count = end.depth
    candidate.steering, candidate.steering_poses = {}, {}
    candidate.steering_poses[0] = {nodes[0].p, nodes[0].f}
    cursor := chosen
    for k := end.depth-1; k >= 0; k -= 1 {
        node := nodes[cursor]
        candidate.steering[k] = node.curvature
        candidate.steering_poses[k+1] = {node.p, node.f}
        cursor = node.parent
    }
    candidate.arc = {}
    for k in 0..<len(candidate.arc) { candidate.arc[k] = f32(min(k, end.depth))*STEERING_STEP }
    candidate.length = f32(end.depth)*STEERING_STEP
    if complete {
        candidate.end = (end.p.x-origin.x)*axis.x+(end.p.y-origin.y)*axis.y
        candidate.lateral = -(end.p.x-origin.x)*axis.y+(end.p.y-origin.y)*axis.x
    }
    return candidate
}

steering_goal :: proc(p, f, origin, axis: Vec2, minimum, maximum, width: f32, kind: Vehicle_Kind) -> (distance: f32, reached: bool) {
    along := (p.x-origin.x)*axis.x+(p.y-origin.y)*axis.y
    lateral := -(p.x-origin.x)*axis.y+(p.y-origin.y)*axis.x
    angle := abs(math.atan2(f.x*axis.y-f.y*axis.x, f.x*axis.x+f.y*axis.y))
    signed_side := lateral-clamp(lateral, -(width-kind_width(kind))*0.5, (width-kind_width(kind))*0.5)
    side := abs(signed_side)
    ahead := max(0, minimum-along)
    // Include turning toward the receiving strip, not only its final heading.
    // Otherwise a north-facing driver west of a northbound strip is rewarded
    // for driving farther north into a dead end instead of correcting east.
    desired := Vec2{axis.x*ahead+axis.y*signed_side, axis.y*ahead-axis.x*signed_side}
    turns := angle
    if side > 0 || ahead > 0 {
        turns = max(angle, abs(math.atan2(f.x*desired.y-f.y*desired.x, f.x*desired.x+f.y*desired.y))+
            abs(math.atan2(desired.x*axis.y-desired.y*axis.x, desired.x*axis.x+desired.y*axis.y)))
    }
    distance = math.sqrt(side*side+ahead*ahead)+STEERING_SPEED*turns/STEERING_YAW_RATE
    reached = along >= minimum && along <= maximum && side == 0 && angle < 0.00001
    return
}

steering_action_clear :: proc(w: ^World, node, incoming, exit: int, incoming_axis: Vec2, kind: Vehicle_Kind, p, f: Vec2, curvature: f32, hull: []Vec2, obstacles: []Steering_Obstacle, samples: int, origin, axis: Vec2, maximum: f32) -> bool {
    for sample in 1..=samples {
        q, g := curvature_pose(p, f, curvature, STEERING_STEP*f32(sample)/f32(samples))
        if (q.x-origin.x)*axis.x+(q.y-origin.y)*axis.y > maximum { return false }
        if incoming == exit && g.x*axis.x+g.y*axis.y <= 0 { return false }
        permitted_incoming := incoming
        if g.x*incoming_axis.x+g.y*incoming_axis.y <= 0 { permitted_incoming = exit }
        // A turning body may straddle the mouth while its centre is still in
        // the junction. Once its whole footprint has left, use only its route's
        // directed strips, not an unrelated/opposing arm of the pavement union.
        radius := math.sqrt(kind_length(kind)*kind_length(kind)+kind_width(kind)*kind_width(kind))*0.5
        if point_in_junction_hull(q, hull, radius) { permitted_incoming = -1 }
        if !body_on_junction_pavement(w, node, q, g, kind, hull, permitted_incoming, exit) { return false }
        for other in obstacles {
            if body_poses_overlap(q, g, kind, other.p, other.f, other.kind) { return false }
        }
    }
    return true
}
