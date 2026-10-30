package sim

import "core:testing"
import "core:fmt"

@(test)
test_connector_straight_left_right_continuous :: proc(t: ^testing.T) {
    ends := []Vec2{{60,0}, {0,60}, {0,-60}}
    kinds := []Vehicle_Kind{.Car, .Bus, .Motorcycle}
    for end in ends {
        for kind in kinds {
            w: World
            init(&w, []Node{{{-60,0}}, {{0,0}}, {end}}, []Road{{0,1,4,8}, {1,0,4,8}, {1,2,5,8}, {2,1,5,8}})
            // Four-arm fixture matching showcase; tight two-arm bus bends are
            // diagnosed separately, never treated as pavement-contained here.
            for arm in ends {
                if arm == end { continue }
                node := len(w.nodes)
                append(&w.nodes, Node{arm})
                append(&w.roads, Road{1,node,4,8}, Road{node,1,4,8})
            }
            testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,kind}}))
            append(&w.vehicles, Vehicle{road = 0, distance = 30, speed = 4, kind = kind, id = 1})
            w.spawned, w.requested = 1, 1
            previous, forward := vehicle_pose(&w, w.vehicles[0])
            saw_connector, saw_exit := false, false
            for _ in 0..<800 {
                old := w.vehicles[0]
                step(&w, 0.05)
                if len(w.vehicles) == 0 {
                    // Terminal trips retain the existing center-at-end removal
                    // convention; no early disappearance on a blocked connector.
                    remaining := road_length(&w, 2)-old.distance
                    if old.motion.active { remaining = old.motion.length-old.motion.progress }
                    testing.expect(t, old.road == 2 && remaining <= kind_speed(kind)*0.05+0.015)
                    break
                }
                v := w.vehicles[0]
                p, f := vehicle_pose(&w, v)
                testing.expect(t, vec_distance(previous, p) <= kind_speed(kind)*0.05+0.015, "position must follow the simulated path without a link teleport")
                testing.expect(t, forward.x*f.x+forward.y*f.y > 0.98, "heading must be continuous at both connector boundaries")
                testing.expect(t, abs(f.x*f.x+f.y*f.y-1) < 0.0001)
                if v.motion.active {
                    start, sf := connector_pose(v.motion, 0)
                    finish, ef := connector_pose(v.motion, v.motion.length)
                    link := v
                    link.motion = {}
                    a, af := vehicle_pose(&w, link)
                    link.distance = v.motion.end
                    if v.motion.exit >= 0 { link.road, link.lateral = v.motion.exit, v.motion.lateral }
                    b, bf := vehicle_pose(&w, link)
                    testing.expect(t, vec_distance(start, a) < 0.0001 && vec_distance(finish, b) < 0.0001)
                    testing.expect(t, vec_distance(sf, af) < 0.0001 && vec_distance(ef, bf) < 0.0001)
                }
                saw_connector = saw_connector || v.motion.active
                saw_exit = saw_exit || v.road == 2
                previous, forward = p, f
            }
            testing.expect(t, saw_connector && saw_exit && w.completed == 1, fmt.tprintf("kind=%v end=%v issues=%v", kind, end, w.motion_issues[:]))
            testing.expect(t, len(w.motion_issues) == 0)
            destroy(&w)
        }
    }
}

@(test)
test_clearance_includes_wider_crossing_strip :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}}, []Road{{0,1,2,8}, {1,2,2,8}, {3,1,16,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Car}}))
    c, problem := make_connector(&w, Vehicle{road = 0, kind = .Car}, 1, 0)
    testing.expect(t, problem == .None)
    testing.expect(t, c.end-kind_length(.Car)*0.5 >= 9, "rear must clear the 16m crossing strip, not just the 2m exit mouth")
    testing.expect(t, junction_clearance_distance(&w, 1) > junction_mouth_depth(&w, 1)+1)
}

@(test)
test_degenerate_uturn_and_short_exit_are_explicit :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{3,0}}, {{0,0}}}, []Road{{0,1,3,8}, {1,0,3,8}, {1,2,3,8}, {1,3,3,8}})
    defer destroy(&w)
    v := Vehicle{road = 0, kind = .Car, spawn_index = -1}
    _, problem := make_connector(&w, v, 1, 0)
    testing.expect(t, problem == .U_Turn)
    _, problem = make_connector(&w, v, 2, 0)
    testing.expect(t, problem == .Short_Exit)
    _, problem = make_connector(&w, v, 3, 0)
    testing.expect(t, problem == .Degenerate)
}
