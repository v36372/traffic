package sim

import "core:math"

// Position and unit forward vector in east/north world coordinates, including
// the directed carriageway offset. Invalid or zero-length roads have no pose.
vehicle_pose :: proc(w: ^World, v: Vehicle) -> (position, forward: Vec2) {
    if v.motion.active { return connector_pose(v.motion, v.motion.progress) }
    if v.road < 0 || v.road >= len(w.roads) { return }
    road := w.roads[v.road]
    length := road_length(w, v.road)
    if length <= 0 { return }
    a, b := w.nodes[road.from].position, w.nodes[road.to].position
    forward = {(b.x-a.x)/length, (b.y-a.y)/length}
    lateral := v.lateral+road_offset(w, v.road)
    distance := clamp(v.distance, 0, length)
    position = {a.x+forward.x*distance-forward.y*lateral, a.y+forward.y*distance+forward.x*lateral}
    return
}

// Match the renderer: reciprocal directed strips occupy opposite sides of the axis.
road_offset :: proc(w: ^World, road: int) -> f32 {
    if len(w.road_offsets) == len(w.roads) { return w.road_offsets[road] }
    r := w.roads[road]
    for other in w.roads {
        if other.from == r.to && other.to == r.from { return -r.width*0.5 }
    }
    return 0
}

junction_mouth_depth :: proc(w: ^World, road: int) -> f32 {
    return min(road_length(w, road)*0.25, w.roads[road].width+abs(road_offset(w, road)))
}

// Front distance from the approach start. The hull's extreme upstream projection
// is the maximum of its mouth and endpoint corners. For each other incident
// strip, also bound its overlap with the approach's full width and pole (0.45m
// beyond the curb). Intersect the infinite-strip bound with the finite rectangle
// projection; the latter keeps near-parallel roads bounded by their actual ends.
// One metre clears the signal head and stop-bar thickness. Only shared node IDs
// participate, matching the rendered junction topology.
CROSSING_DEPTH :: f32(4)
CROSSING_LANDING_DEPTH :: f32(0.45) // 0.4m curb landing plus 0.05m clearance.

// Axis-relative envelope, not relative to this directed strip's offset.
carriageway_span :: proc(w: ^World, road: int) -> (low, high: f32) {
    r := w.roads[road]
    offset := road_offset(w, road)
    low, high = offset-r.width*0.5, offset+r.width*0.5
    for other, i in w.roads {
        if other.from == r.to && other.to == r.from {
            reverse_offset := -road_offset(w, i)
            low = min(low, reverse_offset-other.width*0.5)
            high = max(high, reverse_offset+other.width*0.5)
        }
    }
    return
}

// Full reciprocal-carriageway band. Distances follow the incoming road axis;
// lateral center already includes both directed strips (do not add offset).
// The 0.4m curb landing on either side also clears incident skew strips.
crossing_band :: proc(w: ^World, road: int) -> (near, far, center_lateral, width: f32, ok: bool) {
    return crossing_band_for_signals(w, road, w.signals[:])
}

crossing_band_for_signals :: proc(w: ^World, road: int, signals: []Signal_Plan) -> (near, far, center_lateral, width: f32, ok: bool) {
    length := road_length(w, road)
    low, high := carriageway_span(w, road)
    center_lateral, width = (low+high)*0.5, high-low
    far = length-junction_setback(w, road, false, true)-1
    near = far-CROSSING_DEPTH
    ok = length > 0 && width > 0 && near > 1
    // Adjacent junction crossing envelopes must not overlap. A boundary/source
    // node with no other arm has no second crossing to reserve.
    r := w.roads[road]
    for plan in signals {
        if plan.node == r.from {
            ok = ok && near > junction_setback(w, road, true, true)+1+CROSSING_DEPTH
            break
        }
    }
    return
}

stop_line_distance :: proc(w: ^World, road: int) -> f32 {
    return stop_line_for_signals(w, road, w.signals[:])
}

stop_line_for_signals :: proc(w: ^World, road: int, signals: []Signal_Plan) -> f32 {
    length := road_length(w, road)
    if length <= 0 { return 0 }
    for plan in signals {
        if plan.node == w.roads[road].to {
            near, _, _, _, _ := crossing_band_for_signals(w, road, signals)
            return near-1
        }
    }
    // Unauthored/uncontrolled OSM shape nodes keep the pre-crossing entry.
    return length-junction_setback(w, road, false)-1
}

// Explicit boundary abstraction: only an uncontrolled, single-arm endpoint.
// A destination at an internal junction is not automatically an external exit.
terminal_is_external :: proc(w: ^World, road: int) -> bool {
    if signal_state(w, road) != .Uncontrolled { return false }
    r := w.roads[road]
    for other in w.roads {
        if other.from == r.to && other.to != r.from { return false }
        if other.to == r.to && other.from != r.from { return false }
    }
    return true
}

// Keep center-at-end terminal removal, but never remove a body whose rear still
// occupies an authored crossing. Do not extend it off pavement to clear paint.
terminal_clearance_ok :: proc(w: ^World, road: int, kind: Vehicle_Kind) -> bool {
    if signal_state(w, road) == .Uncontrolled { return true }
    _, far, _, _, _ := crossing_band(w, road)
    return road_length(w, road)-kind_length(kind)*0.5 > far+0.5
}

// Center-path turn entry is the mouth, never the upstream signal stop bar.
turn_start_distance :: proc(w: ^World, road: int) -> f32 {
    return road_length(w, road)-junction_mouth_depth(w, road)
}

// Outgoing distance at which a REAR has cleared all incident strips/hull, not
// merely this narrow road's own mouth. Same authority as approach stop geometry.
junction_clearance_distance :: proc(w: ^World, road: int) -> f32 {
    return junction_setback(w, road, true)+1
}

junction_setback :: proc(w: ^World, road: int, at_start: bool, full_span: bool = false) -> f32 {
    r := w.roads[road]
    length := road_length(w, road)
    if length <= 0 { return 0 }
    from, to := r.from, r.to
    if at_start { from, to = to, from }
    a, node := w.nodes[from].position, w.nodes[to].position
    fx, fy := (node.x-a.x)/length, (node.y-a.y)/length
    lx, ly := -fy, fx
    offset := road_offset(w, road)
    if at_start { offset = -offset }
    lateral := abs(offset)+r.width*0.5+0.45
    if full_span {
        low, high := carriageway_span(w, road)
        lateral = max(abs(low), abs(high))+CROSSING_LANDING_DEPTH
    }
    setback: f32 = 0
    for other, i in w.roads {
        if other.from != to && other.to != to { continue }
        n := road_length(w, i)
        if n <= 0 { continue }
        start, end := w.nodes[other.from].position, w.nodes[other.to].position
        dx, dy := (end.x-start.x)/n, (end.y-start.y)/n
        nx, ny := -dy, dx
        outward_x, outward_y := dx, dy
        if other.to == to { outward_x, outward_y = -dx, -dy }
        // Positive projection points from the junction toward the approach start.
        along := -(outward_x*fx+outward_y*fy)
        side := -(nx*fx+ny*fy)
        off := road_offset(w, i)
        half := other.width*0.5
        setback = max(setback, max(0, junction_mouth_depth(w, i)*along)+off*side+half*abs(side))
        // Reciprocal/same-axis roads are the approach itself, not crossing strips.
        neighbor := other.to
        if other.to == to { neighbor = other.from }
        cross := fx*nx+fy*ny
        if neighbor == from || abs(cross) < 0.00001 { continue }
        // A point at approach lateral coordinate y has strip-normal coordinate
        // -setback*cross + y*dot(left, other_left). Solve for the furthest overlap.
        strip_bound := (half+abs(off)+lateral*abs(lx*nx+ly*ny))/abs(cross)
        end_bound := max(0, n*along)+off*side+half*abs(side)
        setback = max(setback, min(strip_bound, end_bound))
    }
    return setback
}

// Geometry is static during a run, like configured routes. Rebuild the world
// after node/width edits; speed-limit edits need no rebuild.
cache_geometry :: proc(w: ^World) {
    clear(&w.fit_cache)
    clear(&w.road_offsets)
    clear(&w.stop_lines)
    for _, i in w.roads { append(&w.road_offsets, road_offset(w, i)) }
    for _, i in w.roads { append(&w.stop_lines, stop_line_distance(w, i)) }
}
