package sim

import "core:math"
import "core:testing"

@(test)
test_stop_line_geometry :: proc(t: ^testing.T) {
    nodes := []Node{{{-50,0}}, {{0,0}}, {{50,0}}, {{0,-50}}, {{0,50}}, {{-50,-50}}, {{50,50}}, {{0,-10}}, {{0,10}}}
    roads := []Road{
        {0,1,3.5,8}, {1,0,3.5,8}, {1,2,3.5,8},
        {3,1,3.5,8}, {1,3,3.5,8}, {1,4,3.5,8}, {4,1,3.5,8},
    }
    w: World
    init(&w, nodes, roads)
    defer destroy(&w)
    testing.expect(t, road_offset(&w, 0) == -1.75 && road_offset(&w, 2) == 0)
    testing.expect(t, junction_mouth_depth(&w, 0) == 5.25)
    orthogonal := stop_line_distance(&w, 0)
    testing.expect(t, 50-orthogonal >= 6.25)
    // Unequal reciprocal widths affect each strip independently, including hull corners.
    w.roads[4].width = 8
    testing.expect(t, road_offset(&w, 3) == -1.75 && road_offset(&w, 4) == -4)
    wider := stop_line_distance(&w, 0)
    testing.expect(t, wider < orthogonal && 50-wider >= 9)
    // Unconnected geometric crossings must not affect this junction.
    append(&w.roads, Road{7,8,40,8})
    testing.expect(t, stop_line_distance(&w, 0) == wider)
    // A 45-degree reciprocal crossing extends beyond the mouth hull at the pole.
    w.roads[3] = Road{5,1,3.5,8}
    w.roads[4] = Road{1,5,3.5,8}
    w.roads[5] = Road{1,6,3.5,8}
    w.roads[6] = Road{6,1,3.5,8}
    diagonal := stop_line_distance(&w, 0)
    expected_strip := f32(3.5)*math.sqrt(f32(2)) + 3.95
    testing.expect(t, 50-diagonal >= expected_strip+0.999)
    testing.expect(t, diagonal < orthogonal)
}

@(test)
test_vehicle_pose_uses_directed_carriageway :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0, 0}}, {{10, 0}}}, []Road{{0, 1, 4, 5}, {1, 0, 4, 5}})
    defer destroy(&w)
    p, f := vehicle_pose(&w, Vehicle{road = 0, distance = 3, lateral = 0.5})
    testing.expect(t, p == Vec2{3, -1.5} && f == Vec2{1, 0})
    p, f = vehicle_pose(&w, Vehicle{road = 1, distance = 3, lateral = 0.5})
    testing.expect(t, p == Vec2{7, 1.5} && f == Vec2{-1, 0})
}

@(test)
test_link_endpoint_frames_require_a_connector :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-20, 0}}, {{0, 0}}, {{0, 20}}}, []Road{
        {0, 1, 4, 5}, {1, 0, 4, 5}, {1, 2, 4, 5}, {2, 1, 4, 5},
    })
    defer destroy(&w)
    before, _ := vehicle_pose(&w, Vehicle{road = 0, distance = 20})
    after, _ := vehicle_pose(&w, Vehicle{road = 2, distance = 0})
    // Raw link frames differ. Live motion must join them with the connector,
    // whose position/heading continuity is tested in motion_test.odin.
    testing.expect(t, before == Vec2{0, -2} && after == Vec2{2, 0})
}
