package sim

import "core:testing"

@(test)
test_feedback_motion_and_queries :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    w.vehicles[0].id = 71
    w.vehicles[0].distance = 34
    w.vehicles[0].speed = 0
    before := w.vehicles[1].driver
    blocker := u64(999)
    testing.expect(t, !body_clear(&w, w.vehicles[0], 1, &blocker) && blocker == 71)
    w.vehicles[0].distance = 55
    testing.expect(t, body_clear(&w, w.vehicles[1], 1, &blocker) && blocker == 0)
    w.vehicles[0].distance = 34
    feedback: Driver_Feedback
    moved, finished := move_vehicle(&w, 1, 5, &feedback)
    testing.expect(t, moved > 0 && !finished && feedback.moved == moved && feedback.partial)
    testing.expect(t, feedback.reason == .Body && feedback.blocker == 71)
    testing.expect(t, w.vehicles[1].driver == before)
}

@(test)
test_feedback_fit_signal_and_boundary :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{40,0}}, {{120,0}}}, []Road{{0,1,6,8}, {1,2,2,8}})
    defer destroy(&w)
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Car), lateral = 2, preferred_lateral = 2, kind = .Car})
    before := w.vehicles[0].driver
    feedback: Driver_Feedback
    testing.expect(t, !acquire_connector(&w, 0, 1, &feedback))
    testing.expect(t, feedback.attempted && !feedback.accepted && feedback.reason == .Geometry && feedback.problem == .Tight_Turn)
    testing.expect(t, w.vehicles[0].driver == before)
    append(&w.signals, Signal_Plan{node = 1, green = 1, amber = 1, clearance = 100, offset = 2})
    cache_geometry(&w)
    w.vehicles[0].distance = entry_distance(&w, 0, .Car)
    _, _ = move_vehicle(&w, 0, 1, &feedback)
    testing.expect(t, feedback.reason == .Signal && feedback.moved == 0)
    try_overtake(&w, 0, &feedback)
    testing.expect(t, feedback.reason == .Route_Boundary)
    testing.expect(t, w.vehicles[0].driver == before)
}

@(test)
test_feedback_rear_and_prediction :: proc(t: ^testing.T) {
    w: World
    passing_world(&w, false)
    defer destroy(&w)
    w.vehicles[0].id = 42
    w.vehicles[0].distance = 28
    w.vehicles[0].lateral = 2
    w.vehicles[0].speed = 9
    v := w.vehicles[1]
    v.lane = {active = true, start = v.distance, from = 0, to = 2, length = 12}
    blocker := u64(999)
    testing.expect(t, !lane_rear_safe(&w, 1, v, true, &blocker) && blocker == 42)
    w.vehicles[0].distance = 40
    w.vehicles[0].lateral = 0
    w.vehicles[0].speed = 0
    testing.expect(t, !local_gap_clear(&w, w.vehicles[1], 1, 8, 1.5, &blocker) && blocker == 42)
    before := w.vehicles[1].driver
    feedback: Driver_Feedback
    _ = try_local_bypass(&w, 1, &feedback)
    try_overtake(&w, 1, &feedback)
    testing.expect(t, w.vehicles[1].driver == before)
}
