package replay

import "core:math"
import sim "../sim"

// A debugger oracle, independent of the driver's spline planner. A short forward
// arc proves available local movement, not a complete route or future reservation.
// Synthetic radii/horizon are deliberately bounded; use a trajectory search if a
// more complex escape must be diagnosed. This never moves or modifies a driver.
Steering_Probe :: struct { start, forward: sim.Vec2, curvature, distance: f32 }

probe_pose :: proc(probe: Steering_Probe, distance: f32) -> (p, f: sim.Vec2) {
    p, f = probe.start, probe.forward
    k := probe.curvature
    if k == 0 { return {p.x+f.x*distance, p.y+f.y*distance}, f }
    angle := k*distance
    s, c := math.sin(angle), math.cos(angle)
    return {p.x+(f.x*s+f.y*(c-1))/k, p.y+(f.y*s-f.x*(c-1))/k}, {f.x*c-f.y*s, f.y*c+f.x*s}
}

steering_probe :: proc(w: ^sim.World, selected: u64, distance: f32 = 0.75) -> (probe: Steering_Probe, available: bool) {
    if !finite(distance) || distance < 0.25 || distance > 5 { return }
    for v, index in w.vehicles {
        if v.id != selected { continue }
        if v.speed > 0.05 || (v.motion.active && v.motion.progress < v.motion.lead) { return }
        p, f := sim.vehicle_pose(w, v)
        origin, goal := sim.vehicle_pose(w, sim.Vehicle{road = v.road})
        node := w.roads[v.road].to
        hull: [dynamic]sim.Vec2
        if v.motion.active && !v.motion.local {
            node, goal = v.motion.node, v.motion.outgoing
            if v.motion.exit >= 0 { _, goal = sim.vehicle_pose(w, sim.Vehicle{road = v.motion.exit}) }
            hull = sim.junction_polygon(w, node)
        }
        defer delete(hull)
        poses := make([][2]sim.Vec2, len(w.vehicles))
        defer delete(poses)
        for other, j in w.vehicles { poses[j][0], poses[j][1] = sim.vehicle_pose(w, other) }
        for curvature in ([]f32{0, -0.25, 0.25, -0.4, 0.4, -1.0/1.5, 1.0/1.5, -0.125, 0.125}) {
            candidate := Steering_Probe{p, f, curvature, distance}
            end, _ := probe_pose(candidate, candidate.distance)
            if (end.x-p.x)*goal.x+(end.y-p.y)*goal.y < candidate.distance*0.5 { continue }
            clear := true
            samples := int(math.ceil(distance/0.05))
            for step in 0..=samples {
                q, g := probe_pose(candidate, distance*f32(step)/f32(samples))
                permitted := -1
                if !v.motion.active || v.motion.local {
                    permitted = v.road
                    // Ordinary link motion cannot bypass its entry/signal gate.
                    if (q.x-origin.x)*goal.x+(q.y-origin.y)*goal.y > sim.entry_distance(w, v.road, v.kind) { clear = false; break }
                }
                if !sim.body_on_junction_pavement(w, node, q, g, v.kind, hull[:], permitted, permitted) { clear = false; break }
                for other, j in w.vehicles {
                    if j == index { continue }
                    op, of := poses[j][0], poses[j][1]
                    if sim.body_poses_overlap(q, g, v.kind, op, of, other.kind) { clear = false; break }
                }
                if !clear { break }
            }
            if clear { return candidate, true }
        }
        return
    }
    return
}
