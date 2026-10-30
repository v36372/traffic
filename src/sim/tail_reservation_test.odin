package sim

import "core:testing"

@(test)
test_curve_release_requires_clear_remaining_envelope :: proc(t: ^testing.T) {
    straight := Connector{start = {0,0}, controls = [4]Vec2{{0,0},{2,0},{8,0},{10,0}}, outgoing = {1,0}, tail = 1}
    turn := Connector{start = {-5,0}, controls = [4]Vec2{{-5,0},{-3,0},{0,3},{0,5}}, outgoing = {0,1}, lead = 2, tail = 4}
    turn.arc[64] = 8
    turn.length = 14
    turn.progress = 10
    testing.expect(t, remaining_corridor_disjoint(straight, .Motorcycle, turn, .Motorcycle))
    before := turn; before.progress = 9.99
    testing.expect(t, remaining_corridor_disjoint(straight, .Motorcycle, before, .Motorcycle), "clear remainder of curve may release before tail")
    crossing := turn; crossing.progress = 0
    testing.expect(t, !remaining_corridor_disjoint(straight, .Motorcycle, crossing, .Motorcycle), "remaining curve crossing the corridor must block")
    separated_lead := turn
    separated_lead.start.y = 5
    for &point in separated_lead.controls { point.y += 5 }
    separated_lead.progress = 0
    testing.expect(t, remaining_corridor_disjoint(straight, .Motorcycle, separated_lead, .Motorcycle), "clear lead and full curve may release")
    mirrored := turn
    mirrored.start.y = -mirrored.start.y
    mirrored.outgoing.y = -mirrored.outgoing.y
    for &point in mirrored.controls { point.y = -point.y }
    testing.expect(t, remaining_corridor_disjoint(straight, .Motorcycle, mirrored, .Motorcycle), "clearance works on either side")
    far_straight, far_turn := straight, turn
    far_straight.start.x += 1e7
    for &point in far_straight.controls { point.x += 1e7 }
    far_turn.start.x += 1e7
    for &point in far_turn.controls { point.x += 1e7 }
    testing.expect(t, !remaining_corridor_disjoint(far_straight, .Motorcycle, far_turn, .Motorcycle), "large-coordinate rounding margin must fail closed")
    conflict := turn; conflict.controls[3].y = 1
    testing.expect(t, !remaining_corridor_disjoint(straight, .Motorcycle, conflict, .Motorcycle), "body envelope near the corridor must block")
    inward := turn; inward.outgoing = {0,-1}; inward.tail = 1; inward.length = 11
    testing.expect(t, !remaining_corridor_disjoint(straight, .Motorcycle, inward, .Motorcycle), "past the tail an inward outgoing road could enter the corridor")
    curved := straight; curved.controls[1].y = 0.001
    testing.expect(t, !remaining_corridor_disjoint(curved, .Motorcycle, turn, .Motorcycle), "candidate must be exactly straight")
    no_tail := turn; no_tail.tail = 0
    testing.expect(t, !remaining_corridor_disjoint(straight, .Motorcycle, no_tail, .Motorcycle))
    parallel := straight; parallel.progress = 10; parallel.length = 11
    testing.expect(t, !remaining_corridor_disjoint(straight, .Motorcycle, parallel, .Motorcycle), "straight owners use only the existing straight-band rule")
}
