package sim

import "core:fmt"
import "core:testing"

source_fit_world :: proc(w: ^World) {
    // Real regression: west 7m carriageways -> south 2m carriageways.
    init(w, []Node{{{0,0}}, {{-120,0}}, {{120,0}}, {{0,120}}, {{0,-120}}}, []Road{
        {1,0,7,9}, {0,1,7,9}, {0,2,7,9}, {2,0,7,9},
        {3,0,2,9}, {0,3,2,9}, {0,4,2,9}, {4,0,2,9},
    })
}

@(test)
test_source_and_node_admission_try_noncentral_receiving_bands :: proc(t: ^testing.T) {
    w: World
    source_fit_world(&w)
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,4,0.05,.Car}}))
    v := Vehicle{road = 0, kind = .Car}
    _, center := make_connector(&w, v, 6, 0)
    edge := (w.roads[6].width-kind_width(.Car))*0.5
    c, problem := make_connector(&w, v, 6, edge)
    testing.expect(t, center == .Tight_Turn && problem == .None)
    testing.expect(t, source_supported(&w, v) && len(w.motion_issues) == 0,
        "a failed center-exit trial must not reject a valid receiving band")
    step(&w, 0.05)
    testing.expect(t, w.spawned == 1 && w.pending == 0 && len(w.motion_issues) == 0)
    w.vehicles[0].distance = entry_distance(&w, 0, .Car)
    owner := w.vehicles[0]
    owner.id, owner.motion = 2, c
    owner.motion.progress = c.length-0.1
    append(&w.vehicles, owner)
    w.spawned, w.requested = 2, 2
    testing.expect(t, acquire_connector(&w, 0, 6) && len(w.motion_issues) == 0,
        "a distant connector remainder does not reserve the candidate's local conflict window")
    testing.expect(t, w.vehicles[0].motion.active && w.vehicles[1].motion.active)
}

@(test)
test_source_validates_actual_fitted_incoming_slot :: proc(t: ^testing.T) {
    w: World
    source_fit_world(&w)
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,4,0.05,.Car}}))
    w.roads[0].speed_limit = 0
    append(&w.vehicles, Vehicle{road = 0, kind = .Bicycle, id = 1})
    w.spawned, w.requested = 1, 1
    actual, fits := fitting_lateral(&w, 0, .Car, 0)
    testing.expect(t, fits && actual != 0)
    center := source_geometry_problem(&w, Vehicle{road = 0, kind = .Car})
    fitted := source_geometry_problem(&w, Vehicle{road = 0, kind = .Car, lateral = actual})
    testing.expect(t, center == .None && fitted == .Tight_Turn,
        fmt.tprintf("actual incoming=%g center=%v actual=%v", actual, center, fitted))
    step(&w, 0.05)
    testing.expect(t, w.spawned == 1 && len(w.vehicles) == 1 && w.pending == 1 && w.requested == 2,
        "must retain demand instead of admitting a bad actual slot based on a valid zero-lateral trial")
    testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Tight_Turn)
}
