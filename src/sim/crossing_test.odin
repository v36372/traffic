package sim

import "core:testing"
import "core:fmt"

@(test)
test_crossing_full_reciprocal_span_clears_skew_strips_and_landings :: proc(t: ^testing.T) {
    for fixture in ([]struct { skew: bool, widths: [4]f32 }{{false,{4,4,4,4}}, {false,{3,6,9,4}}, {true,{4,3,8,2}}}) {
        arm := Vec2{0,60}
        if fixture.skew { arm = {42,42} }
        x := fixture.widths
        w: World
        init(&w, []Node{{{0,0}}, {{-60,0}}, {{60,0}}, {arm}, {{-arm.x,-arm.y}}}, []Road{
            {1,0,x[0],8}, {0,1,x[1],8}, {0,2,x[1],8}, {2,0,x[0],8},
            {3,0,x[2],8}, {0,3,x[3],8}, {0,4,x[3],8}, {4,0,x[2],8},
        })
        testing.expect(t, configure(&w, []Signal_Plan{{0,100,1,1,0, false}}, nil))
        near, far, center, width, ok := crossing_band(&w, 0)
        testing.expect(t, ok && far-near == 4 && stop_line_distance(&w, 0) == near-1)
        testing.expect(t, width == x[0]+x[1] && center == (x[1]-x[0])*0.5)
        testing.expect(t, far < turn_start_distance(&w, 0))
        // Independently sample the full paint+0.4m outside curb landings against
        // finite incident road rectangles, not just this approach's own strip.
        for k in 0..=16 {
            distance := near+(far-near)*f32(k)/16
            for j in 0..=64 {
                lateral := center-width*0.5-0.4+(width+0.8)*f32(j)/64
                point := Vec2{-60+distance, lateral}
                for road, road_id in w.roads {
                    if road.from == 1 || road.to == 1 { continue } // own reciprocal arm
                    length := road_length(&w, road_id)
                    a, b := w.nodes[road.from].position, w.nodes[road.to].position
                    f := Vec2{(b.x-a.x)/length, (b.y-a.y)/length}
                    along := (point.x-a.x)*f.x+(point.y-a.y)*f.y
                    side := -(point.x-a.x)*f.y+(point.y-a.y)*f.x-road_offset(&w, road_id)
                    inside := along >= 0 && along <= length && abs(side) <= road.width*0.5
                    testing.expect(t, !inside, "crossing/landing must stay outside ALL incident skew strips")
                }
            }
        }
        destroy(&w)
    }
}

@(test)
test_reservation_at_bar_straight_lead_through_band_then_mouth_turn :: proc(t: ^testing.T) {
    for kind in ([]Vehicle_Kind{.Motorcycle, .Car, .Bus}) {
        w: World
        init(&w, []Node{{{-60,0}}, {{0,0}}, {{0,60}}, {{60,0}}, {{0,-60}}}, []Road{
            {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8}, {1,3,4,8}, {3,1,4,8}, {1,4,4,8}, {4,1,4,8},
        })
        testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0, false}}, []Spawn_Point{{0,2,86400,kind}}))
        entry := entry_distance(&w, 0, kind)
        near, far, _, _, ok := crossing_band(&w, 0)
        testing.expect(t, ok && entry+kind_length(kind)*0.5 < near)
        append(&w.vehicles, Vehicle{road = 0, distance = entry, speed = 2, kind = kind, id = 1})
        w.spawned, w.requested = 1, 1
        step(&w, 0.05)
        v := w.vehicles[0]
        if !testing.expect(t, v.motion.active && v.motion.lead > 0, fmt.tprintf("kind=%v issues=%v", kind, w.motion_issues[:])) {
            destroy(&w)
            continue
        }
        c := v.motion
        testing.expect(t, c.controls[0].x >= -60+turn_start_distance(&w, 0)+kind_length(kind)*0.5-0.0001)
        testing.expect(t, turn_start_distance(&w, 0) > far)
        for k in 0..=100 {
            d := c.lead*f32(k)/100
            p, f := connector_pose(c, d)
            expected, ef := vehicle_pose(&w, Vehicle{road = 0, distance = entry+d, kind = kind})
            testing.expect(t, vec_distance(p, expected) < 0.0001 && vec_distance(f, ef) < 0.0001,
                "heading/lateral must stay exactly on the incoming road through the full pedestrian band")
        }
        at, af := connector_pose(c, c.lead)
        after, bf := connector_pose(c, c.lead+0.001)
        testing.expect(t, vec_distance(at, after) < 0.002 && af.x*bf.x+af.y*bf.y > 0.9999)
        // Change phase while committed on the straight lead, before the band.
        w.signals[0].offset, w.time = 102, 0
        saw_turn := false
        for _ in 0..<800 {
            step(&w, 0.05)
            if len(w.vehicles) == 0 { break }
            current := w.vehicles[0]
            if current.motion.active && current.motion.node == 1 {
                p, f := vehicle_pose(&w, current)
                if p.x+60 <= far { testing.expect(t, abs(f.y) < 0.0001 && abs(p.y+2) < 0.0001) }
                if current.motion.progress > current.motion.lead+0.1 { saw_turn = true }
            }
        }
        testing.expect(t, saw_turn && w.completed == 1 && len(w.motion_issues) == 0)
        destroy(&w)
    }
}

@(test)
test_crossing_short_and_overlapping_layouts_are_explicit :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{16,0}}, {{50,0}}, {{0,-40}}, {{16,-40}}}, []Road{
        {0,1,6,8}, {1,2,6,8}, {3,0,6,8}, {4,1,6,8}, {1,0,6,8},
    })
    defer destroy(&w)
    base := []Signal_Plan{{1,100,1,1,0, false}}
    proposed := []Signal_Plan{{0,100,1,1,0, false}, {1,100,1,1,0, false}}
    testing.expect(t, configure(&w, base, nil))
    old_line := w.stop_lines[0]
    _, _, _, _, ok := crossing_band_for_signals(&w, 0, proposed)
    testing.expect(t, !ok, "opposing authored crossing envelopes must not overlap on a short shared link")
    testing.expect(t, !configure(&w, proposed, nil))
    testing.expect(t, len(w.signals) == 1 && w.signals[0] == base[0] && w.stop_lines[0] == old_line,
        "failed candidate geometry must leave the previous plan/cache untouched")
    short: World
    init(&short, []Node{{{0,0}}, {{3,0}}}, []Road{{0,1,3,8}})
    defer destroy(&short)
    _, _, _, _, ok = crossing_band(&short, 0)
    testing.expect(t, !ok && stop_line_for_signals(&short, 0, []Signal_Plan{{1,100,1,1,0, false}}) < 0)
}

@(test)
test_uncontrolled_geometry_and_flow_ignore_unauthored_crossings :: proc(t: ^testing.T) {
    a, b: World
    nodes := []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{-60,200}}, {{0,200}}, {{0,260}}, {{60,200}}}
    roads := []Road{{0,1,3.5,8}, {1,2,3.5,8}, {3,4,3.5,8}, {5,4,3.5,8}, {4,6,3.5,8}}
    init(&a, nodes, roads, 42)
    init(&b, nodes, roads, 42)
    defer destroy(&a)
    defer destroy(&b)
    source := []Spawn_Point{{0,2,1.2,.Motorcycle}}
    testing.expect(t, configure(&a, nil, source) && configure(&b, []Signal_Plan{{4,100,1,1,0, false}}, source))
    legacy := road_length(&a, 0)-junction_setback(&a, 0, false)-1
    testing.expect(t, stop_line_distance(&a, 0) == legacy && stop_line_distance(&b, 0) == legacy)
    testing.expect(t, a.stop_lines[0] == legacy && b.stop_lines[0] == legacy)
    for _ in 0..<2400 {
        step(&a, 0.05)
        step(&b, 0.05)
        testing.expect(t, a.spawned == b.spawned && a.completed == b.completed && a.pending == b.pending && len(a.vehicles) == len(b.vehicles))
        for v, i in a.vehicles { testing.expect(t, v == b.vehicles[i]) }
    }
    testing.expect(t, a.completed > 0 && len(a.motion_issues) == 0 && len(b.motion_issues) == 0)
}

@(test)
test_tight_outer_curb_fit_is_named_not_silently_admitted :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{40,0}}, {{120,0}}}, []Road{{0,1,6,8}, {1,2,2,8}})
    defer destroy(&w)
    w.spawn_accumulator = -100
    v := Vehicle{road = 0, distance = entry_distance(&w, 0, .Car), lateral = 2, preferred_lateral = 2, kind = .Car}
    append(&w.vehicles, v)
    // Only .05m curb margin: this is a ceiling of the current center-tangent
    // rigid-body/cubic family, NOT proof that a real car cannot make the merge.
    testing.expect(t, !acquire_connector(&w, 0, 1))
    testing.expect(t, !w.vehicles[0].motion.active && w.vehicles[0].distance == v.distance)
    testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Tight_Turn)
}

@(test)
test_fit_cache_is_per_configuration_and_reset :: proc(t: ^testing.T) {
    w: World
    nodes := []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}}
    roads := []Road{{0,1,3.5,8}, {1,2,3.5,8}, {3,1,3.5,8}}
    init(&w, nodes, roads)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}}))
    old, problem := make_connector(&w, Vehicle{road = 0, kind = .Motorcycle}, 1, 0)
    testing.expect(t, problem == .None && len(w.fit_cache) > 0)
    count := len(w.fit_cache)
    testing.expect(t, !configure(&w, []Signal_Plan{{99,2,1,1,0, false}}, nil) && len(w.fit_cache) == count)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0, false}}, []Spawn_Point{{0,2,86400,.Motorcycle}}) && len(w.fit_cache) == 0)
    current, why := make_connector(&w, Vehicle{road = 0, kind = .Motorcycle}, 1, 0)
    testing.expect(t, why == .None && current.start.x < old.start.x && current.lead > old.lead)
    destroy(&w)
    init(&w, nodes, roads)
    testing.expect(t, len(w.fit_cache) == 0 && len(w.motion_issues) == 0 && w.spawned == 0 && w.rng == 1)
    destroy(&w)
}

@(test)
test_terminal_clearance_rejects_short_body_space_without_offmap_extension :: proc(t: ^testing.T) {
    cases := []struct{width: f32, kind: Vehicle_Kind, supported, forward_pavement: bool}{
        {3,.Bus,false,false}, {6,.Bus,false,false}, {3,.Car,false,false},
        {6,.Bus,true,true}, {3,.Car,true,true}, {3,.Bus,false,true},
    }
    for c in cases {
        w: World
        init(&w, []Node{{{0,0}}, {{60,0}}, {{60,60}}, {{120,0}}}, []Road{{0,1,c.width,8}, {2,1,c.width,8}})
        if c.forward_pavement { append(&w.roads, Road{1,3,c.width,8}) }
        testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0, false}}, []Spawn_Point{{0,1,0.05,c.kind}}))
        step(&w, 0.05)
        w.spawns[0].interval, w.spawn_runtime[0].elapsed = 86400, 0
        if !c.supported {
            testing.expect(t, w.spawned == 0 && len(w.vehicles) == 0 && w.pending == 1 && w.requested == 1)
            testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Terminal_Clearance)
            v := Vehicle{road = 0, distance = entry_distance(&w, 0, c.kind), kind = c.kind}
            _, problem := make_connector(&w, v, -1, 0)
            testing.expect(t, problem == .Terminal_Clearance, "runtime and source must enforce the same actual terminal clearance")
            destroy(&w)
            continue
        }
        _, far, _, _, ok := crossing_band(&w, 0)
        testing.expect(t, ok && terminal_clearance_ok(&w, 0, c.kind) && w.spawned == 1)
        hull := junction_polygon(&w, 1)
        finished := false
        for _ in 0..<1000 {
            before := w.vehicles[0]
            p, f := vehicle_pose(&w, before)
            testing.expect(t, p.x <= 60.0001, "do not extend an exit off pavement to clear its painted band")
            if before.motion.active {
                for along in ([]f32{-0.5,0,0.5}) {
                    for across in ([]f32{-0.5,0,0.5}) {
                        if along == 0 && across == 0 { continue }
                        x, y := along*kind_length(c.kind), across*kind_width(c.kind)
                        testing.expect(t, point_on_junction_pavement(&w, 1, Vec2{p.x+f.x*x-f.y*y,p.y+f.y*x+f.x*y}, hull[:]))
                    }
                }
            }
            step(&w, 0.05)
            if len(w.vehicles) == 0 {
                testing.expect(t, p.x-kind_length(c.kind)*0.5 > far,
                    "the preceding fixed-tick rear must already clear the band before completion/release")
                finished = true
                break
            }
        }
        testing.expect(t, finished && w.completed == 1 && w.spawned == 1 && w.pending == 0 && len(w.motion_issues) == 0)
        delete(hull)
        destroy(&w)
    }
}

@(test)
test_receiving_red_terminal_bar_is_not_skipped_by_connector :: proc(t: ^testing.T) {
    for destination in ([]int{2,4}) {
        w: World
        init(&w, []Node{{{-60,0}}, {{0,0}}, {{21,0}}, {{21,-60}}, {{81,0}}}, []Road{
            {0,1,3,8}, {1,2,3,8}, {3,2,3,8}, {2,4,3,8},
        })
        testing.expect(t, configure(&w, []Signal_Plan{{2,100,100,100,200, false}}, []Spawn_Point{{0,destination,0.05,.Bus}}))
        v := Vehicle{road = 0, kind = .Bus}
        testing.expect(t, signal_state(&w, 1) == .Red && entry_distance(&w, 1, .Bus) == 7)
        _, problem := make_connector(&w, v, 1, 0)
        testing.expect(t, problem == .Short_Exit,
            "terminal and continuing trips cannot commit past a downstream controlled stop/crossing")
        step(&w, 0.05)
        testing.expect(t, w.spawned == 0 && len(w.vehicles) == 0 && w.pending == 1 && w.requested == 1)
        testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Short_Exit)
        destroy(&w)
    }
}

@(test)
test_external_terminal_abstraction_is_only_uncontrolled_single_arm :: proc(t: ^testing.T) {
    for internal in ([]bool{false,true}) {
        w: World
        init(&w, []Node{{{0,0}}, {{60,0}}, {{60,60}}}, []Road{{0,1,3,8}})
        if internal { append(&w.roads, Road{2,1,3,8}) }
        testing.expect(t, configure(&w, nil, []Spawn_Point{{0,1,0.05,.Car}}))
        testing.expect(t, terminal_is_external(&w, 0) == !internal)
        step(&w, 0.05)
        w.spawns[0].interval, w.spawn_runtime[0].elapsed = 86400, 0
        if internal {
            testing.expect(t, w.spawned == 0 && w.pending == 1 && len(w.motion_issues) == 1 && w.motion_issues[0].problem == .Terminal_Clearance)
        } else {
            for _ in 0..<1000 { step(&w, 0.05) }
            testing.expect(t, w.completed == 1 && w.spawned == 1 && w.pending == 0 && len(w.motion_issues) == 0)
        }
        destroy(&w)
    }
}
