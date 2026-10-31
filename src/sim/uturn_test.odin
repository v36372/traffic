package sim

import "core:testing"

@(test)
test_car_uturn_stays_explicitly_unsupported :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,60}}, {{0,-60}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8},
        {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
    })
    defer destroy(&w)
    _, problem := make_connector(&w, Vehicle{road = 0, kind = .Car, spawn_index = -1}, 1, 0)
    testing.expect(t, problem == .U_Turn)
}

@(test)
test_motorcycle_uturns_at_junction_and_midblock :: proc(t: ^testing.T) {
    // Four-arm box: west → center, reverse is the U-turn exit.
    w: World
    init(&w, []Node{{{-80,0}}, {{-40,0}}, {{0,0}}, {{80,0}}, {{0,80}}, {{0,-80}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8},
        {2,3,4,8}, {3,2,4,8}, {2,4,4,8}, {4,2,4,8}, {2,5,4,8}, {5,2,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{2,0,86400,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 2, distance = entry_distance(&w, 2, .Motorcycle), kind = .Motorcycle, id = 1})
    testing.expect(t, acquire_connector(&w, 0, 3))
    c := w.vehicles[0].motion
    testing.expect(t, c.active && c.lead < 0.001 && heading_reverses(c.forward, c.outgoing))
    hull := junction_polygon(&w, 2)
    testing.expect(t, connector_on_pavement(&w, c, .Motorcycle, hull[:]))
    delete(hull)
    previous, forward := vehicle_pose(&w, w.vehicles[0])
    for _ in 0..<800 {
        old := w.vehicles[0]
        step(&w, 0.05)
        if len(w.vehicles) == 0 {
            testing.expect(t, old.road == 1 || old.road == 0)
            break
        }
        v := w.vehicles[0]
        p, f := vehicle_pose(&w, v)
        testing.expect(t, vec_distance(previous, p) <= kind_speed(.Motorcycle)*0.05+0.015)
        testing.expect(t, forward.x*f.x+forward.y*f.y > 0.98)
        previous, forward = p, f
    }
    testing.expect(t, w.completed == 1 && len(w.motion_issues) == 0)

    // Mid-block: U-turn at west, before the four-arm.
    m: World
    init(&m, []Node{{{-110,0}}, {{-80,0}}, {{-40,0}}, {{0,0}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8}, {2,3,4,8}, {3,2,4,8},
    })
    defer destroy(&m)
    testing.expect(t, configure(&m, nil, []Spawn_Point{{2,0,86400,.Motorcycle}}))
    testing.expect(t, source_supported(&m, Vehicle{road = 2, kind = .Motorcycle}))
    append(&m.vehicles, Vehicle{road = 2, distance = entry_distance(&m, 2, .Motorcycle), kind = .Motorcycle, id = 1})
    testing.expect(t, acquire_connector(&m, 0, 3))
    testing.expect(t, heading_reverses(m.vehicles[0].motion.forward, m.vehicles[0].motion.outgoing))
    for _ in 0..<800 { step(&m, 0.05); if len(m.vehicles) == 0 { break } }
    testing.expect(t, m.completed == 1 && len(m.motion_issues) == 0)
}

@(test)
test_motorcycle_uturn_source_is_admitted :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-80,0}}, {{-40,0}}, {{0,0}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{2,0,0.05,.Motorcycle}, {2,0,0.05,.Car}}))
    moto := Vehicle{road = 2, kind = .Motorcycle}
    car := Vehicle{road = 2, kind = .Car, spawn_index = 1}
    testing.expect(t, source_supported(&w, moto))
    testing.expect(t, !source_supported(&w, car) && w.motion_issues[0].problem == .U_Turn)
    step(&w, 0.05)
    testing.expect(t, w.spawned == 1 && w.vehicles[0].kind == .Motorcycle)
}

@(test)
test_offset_motorcycle_uturn_tries_other_bands :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-80,0}}, {{-40,0}}, {{0,0}}, {{80,0}}, {{0,80}}, {{0,-80}}}, []Road{
        {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8},
        {2,3,4,8}, {3,2,4,8}, {2,4,4,8}, {4,2,4,8}, {2,5,4,8}, {5,2,4,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{2,0,86400,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 2, distance = entry_distance(&w, 2, .Motorcycle),
        lateral = -0.465, preferred_lateral = -0.465, kind = .Motorcycle, id = 1})
    testing.expect(t, acquire_connector(&w, 0, 3) && w.vehicles[0].motion.active)
    testing.expect(t, heading_reverses(w.vehicles[0].motion.forward, w.vehicles[0].motion.outgoing))
    testing.expect(t, len(w.motion_issues) == 0)
}
