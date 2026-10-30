package app

import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_projection_and_reciprocal_carriageways :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{20, 0}}, {{20, 10}}},
        []sim.Road{{0, 1, 6, 9}, {1, 0, 4, 9}, {1, 2, 3, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    testing.expect(t, offsets[0] == -3 && offsets[1] == -2 && offsets[2] == 0)
    a, forward, left, length := road_frame(&world, world.roads[0])
    testing.expect(t, a == Vec3{0, 0, 0} && forward == Vec3{1, 0, 0})
    testing.expect(t, left == Vec3{0, 0, -1} && length == 20)
    // Eastbound is south/right of the divider; westbound is north/right.
    east_center := a+forward*10+left*offsets[0]
    b, back, west_left, _ := road_frame(&world, world.roads[1])
    west_center := b+back*10+west_left*offsets[1]
    testing.expect(t, east_center == Vec3{10, 0, 3})
    testing.expect(t, west_center == Vec3{10, 0, -2})
    testing.expect(t, east_center[2]-3 == west_center[2]+2)
    _, north, north_left, _ := road_frame(&world, world.roads[2])
    testing.expect(t, north == Vec3{0, 0, -1} && north_left == Vec3{-1, 0, 0})
    // strip's first triangle is CCW as seen from above (+Y).
    side := left*3
    ab, ac := forward*length, forward*length+side*2
    normal_y := ab[2]*ac[0]-ab[0]*ac[2]
    testing.expect(t, normal_y > 0)
}

@(test)
test_fit_view_uses_projected_bounds :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{200, 400}}, {{300, 600}}}, []sim.Road{{0, 1, 6, 9}})
    defer sim.destroy(&world)
    view := fit_view(&world)
    testing.expect(t, view.target == Vec3{250, 0, -500})
    testing.expect(t, math.abs(view.extent-216.4) < 0.01 && view.distance > view.extent)
    view.yaw = 0
    camera := view_camera(view)
    testing.expect(t, math.abs(camera.position[0]-view.target[0]) < 0.001)
    testing.expect(t, camera.position[2] > view.target[2])
    testing.expect(t, camera.position[1] > 0)
}

@(test)
test_failed_interactive_load_preserves_scene :: proc(t: ^testing.T) {
    world: sim.World
    sim.init_demo(&world, 42)
    defer sim.destroy(&world)
    for _ in 0..<100 { sim.step(&world, f32(TICK)) }
    nodes, roads, vehicles := len(world.nodes), len(world.roads), len(world.vehicles)
    clock, spawned := world.time, world.spawned
    testing.expect(t, !load_map(&world, ""))
    testing.expect(t, len(world.nodes) == nodes && len(world.roads) == roads)
    testing.expect(t, len(world.vehicles) == vehicles && world.time == clock && world.spawned == spawned)
}

@(test)
test_empty_scene_view_and_degenerate_frame_are_finite :: proc(t: ^testing.T) {
    world: sim.World
    view := fit_view(&world)
    testing.expect(t, view.extent > 0 && view.distance > 0)
    sim.init(&world, []sim.Node{{{2, 3}}}, []sim.Road{{0, 0, 4, 9}})
    defer sim.destroy(&world)
    _, forward, left, length := road_frame(&world, world.roads[0])
    testing.expect(t, length == 0 && forward == Vec3{} && left == Vec3{})
}
