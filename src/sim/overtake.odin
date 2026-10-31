package sim

import "core:math"

// Lane changes are confined to one straight link and finish before either
// junction envelope. The fixed 12m transition and sampled collision checks
// are conservative gameplay approximations, not swept-volume certification.
Lane_Change :: struct { active: bool, start, from, to: f32 }
LANE_CHANGE_LENGTH :: f32(12)

lane_offset :: proc(v: Vehicle) -> (offset, slope: f32) {
    u := clamp((v.distance-v.lane.start)/LANE_CHANGE_LENGTH, 0, 1)
    delta := v.lane.to-v.lane.from
    return v.lane.from+delta*u*u*(3-2*u), delta*6*u*(1-u)/LANE_CHANGE_LENGTH
}

// The closing-speed horizon covers the remaining transition at current speed
// (floor 1m/s), plus two seconds. Recheck during steering; sampled world-body
// checks still arbitrate actual movement if the prediction changes.
lane_rear_safe :: proc(w: ^World, i: int, v: Vehicle, predictive: bool) -> bool {
    radius := 0.5*math.sqrt(kind_length(v.kind)*kind_length(v.kind)+kind_width(v.kind)*kind_width(v.kind))
    remaining := max(0, v.lane.start+LANE_CHANGE_LENGTH-v.distance)
    horizon := 2+remaining/max(1, v.speed)
    for other, j in w.vehicles {
        if i == j || other.road != v.road || other.distance > v.distance { continue }
        // A straight follower in the occupied band is already held behind by
        // longitudinal following; it must not veto departure indefinitely.
        if !other.motion.active && !other.lane.active && abs(other.lateral-v.lane.from) < (kind_width(other.kind)+kind_width(v.kind))*0.5+0.18 { continue }
        // Once a straight follower is yielding in the currently occupied band,
        // do not let the rear-gap buffer deadlock both vehicles. Actual body
        // checks still protect each movement; admission remains predictive.
        if !predictive && !other.motion.active && !other.lane.active && other.speed <= v.speed && abs(other.lateral-v.lateral) < (kind_width(other.kind)+kind_width(v.kind))*0.5+0.18 { continue }
        other_radius := 0.5*math.sqrt(kind_length(other.kind)*kind_length(other.kind)+kind_width(other.kind)*kind_width(other.kind))
        low, high := min(v.lateral, v.lane.to)-radius, max(v.lateral, v.lane.to)+radius
        other_low, other_high := other.lateral-other_radius, other.lateral+other_radius
        if other.lane.active { other_low = min(other_low, other.lane.to-other_radius); other_high = max(other_high, other.lane.to+other_radius) }
        if other_high < low || other_low > high { continue }
        speed := other.speed
        if predictive {
            target := min(w.roads[other.road].speed_limit, kind_speed(other.kind))
            if w.behavior.enabled { target *= other.driver.traits.speed_factor }
            speed = max(speed, target)
        }
        closing := max(0, speed-v.speed)
        gap := v.distance-other.distance-(kind_length(v.kind)+kind_length(other.kind))*0.5
        if gap < 1.2+closing*horizon { return false }
    }
    return true
}

try_overtake :: proc(w: ^World, i: int) {
    v := w.vehicles[i]
    // Preserve the sampled all-default baseline; explicit non-balanced
    // personas may still pass when attached to a behavior-enabled world.
    varied := false
    for r in w.behavior.classes { if r.speed_min != r.speed_max || r.headway_min != r.headway_max { varied = true; break } }
    if !w.behavior.enabled || (!varied && v.driver.persona == .Balanced) || v.motion.active || v.lane.active { return }
    radius := 0.5*math.sqrt(kind_length(v.kind)*kind_length(v.kind)+kind_width(v.kind)*kind_width(v.kind))
    if v.distance-radius <= junction_setback(w, v.road, true, true)+1+CROSSING_DEPTH { return }
    if v.distance+LANE_CHANGE_LENGTH+radius >= entry_distance(w, v.road, v.kind) { return }
    // Do not start a second maneuver behind one still changing bands.
    for other in w.vehicles { if other.lane.active && other.road == v.road && other.distance >= v.distance && other.distance-v.distance < 35 { return } }
    target := min(w.roads[v.road].speed_limit, kind_speed(v.kind))*v.driver.traits.speed_factor
    lead_index := -1
    gap := f32(35)
    for lead, j in w.vehicles {
        if i == j || lead.road != v.road || lead.motion.active || lead.distance <= v.distance { continue }
        if abs(lead.lateral-v.lateral) >= (kind_width(lead.kind)+kind_width(v.kind))*0.5+0.18 { continue }
        d := lead.distance-v.distance
        if d < gap { lead_index, gap = j, d }
    }
    threshold := passing_advantage(v.driver)
    if lead_index < 0 || w.vehicles[lead_index].lane.active || w.vehicles[lead_index].speed+threshold >= target { return }
    lead := w.vehicles[lead_index]
    shift := (kind_width(lead.kind)+kind_width(v.kind))*0.5+0.3
    side := v.driver.side
    // Up to 121 world-body scans per side per trial, repeated each tick;
    // population is capped at 120. Use a spatial corridor index/candidate
    // cache before scaling beyond that ceiling.
    for sign in ([2]f32{side, -side}) {
        candidate := v
        candidate.lane = {active = true, start = v.distance, from = v.lateral, to = lead.lateral+sign*shift}
        // The cubic's maximum slope is 1.5*shift/length. Projecting the
        // rectangle with sin(angle)<=slope conservatively bounds its corners.
        extent := kind_width(v.kind)*0.5+kind_length(v.kind)*0.5*1.5*abs(candidate.lane.to-candidate.lane.from)/LANE_CHANGE_LENGTH
        if max(abs(candidate.lane.from), abs(candidate.lane.to))+extent > w.roads[v.road].width*0.5 || !lane_rear_safe(w, i, candidate, true) { continue }
        clear := true
        for k in 0..=120 {
            trial := candidate
            trial.distance = v.distance+f32(k)*0.1
            trial.lateral, _ = lane_offset(trial)
            if !body_clear(w, trial, i) || !local_gap_clear(w, trial, i, v.speed) { clear = false; break }
        }
        if clear { w.vehicles[i].lane = candidate.lane; w.vehicles[i].driver.mode = .Passing; return }
    }
}
