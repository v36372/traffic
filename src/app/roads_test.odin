package app

import "core:math"
import "core:testing"
import sim "../sim"

road_triangle_covers :: proc(tri: [3]Vec3, point: Vec3) -> bool {
    return road_turn(tri[0], tri[1], point) >= -0.001 && road_turn(tri[1], tri[2], point) >= -0.001 && road_turn(tri[2], tri[0], point) >= -0.001
}

road_point_covered :: proc(scene: ^Road_Scene, layer: int, point: Vec3) -> bool {
    for tri in scene.layers[layer] {
        if road_triangle_covers(tri, point) { return true }
    }
    return false
}

@(test)
test_shared_junction_fills_offset_carriageway_corners_and_sidewalk :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{-40, 0}}, {{40, 0}}, {{0, 40}}, {{0, -40}}},
        []sim.Road{{0, 1, 3.5, 9}, {1, 0, 3.5, 9}, {0, 2, 3.5, 9}, {2, 0, 3.5, 9}, {0, 3, 3.5, 9}, {3, 0, 3.5, 9}, {0, 4, 3.5, 9}, {4, 0, 3.5, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    scene := build_road_scene(&world, offsets)
    defer destroy_road_scene(&scene)
    testing.expect(t, len(scene.junction_nodes) == 1 && scene.junction_nodes[0] == 0)
    // Outside both rectangular street strips: the joined corner must fill this.
    for x in ([]f32{-4, 4}) {
        for z in ([]f32{-4, 4}) { testing.expect(t, road_point_covered(&scene, 2, {x, 0, z}), "shared junction must not leave green corner wedges") }
    }
    testing.expect(t, road_point_covered(&scene, 0, {20, 0, 5.6}), "2.2m sidewalk outside 3.5m half street")
    testing.expect(t, !road_point_covered(&scene, 2, {20, 0, 4}), "sidewalk must not become asphalt")
    for layer, i in scene.layers {
        for tri in layer {
            testing.expect(t, road_turn(tri[0], tri[1], tri[2]) > 0, "all cached geometry must face upward")
            for p in tri { testing.expect(t, math.abs(p[1]-ROAD_HEIGHT[i]) < 0.0001) }
        }
    }
    destroy_road_scene(&scene)
    testing.expect(t, cap(scene.layers[0]) == 0 && cap(scene.junction_nodes) == 0)
}

@(test)
test_geometric_crossings_never_create_junctions :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{-20, 0}}, {{20, 0}}, {{0, -20}}, {{0, 20}}}, []sim.Road{{0, 1, 4, 9}, {2, 3, 4, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    scene := build_road_scene(&world, offsets)
    defer destroy_road_scene(&scene)
    testing.expect(t, len(scene.junction_nodes) == 0)
    testing.expect(t, len(scene.layers[2]) == 4, "only the original two strips; no synthetic connector")
    testing.expect(t, !road_point_covered(&scene, 2, {3, 0, 3}))
}

@(test)
test_asymmetric_reciprocal_bend_and_short_links_are_bounded :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{-8, 0}}, {{0, 0}}, {{0, 5}}}, []sim.Road{{0, 1, 6, 9}, {1, 0, 4, 9}, {1, 2, 3, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    scene := build_road_scene(&world, offsets)
    defer destroy_road_scene(&scene)
    testing.expect(t, len(scene.junction_nodes) == 1 && scene.junction_nodes[0] == 1)
    testing.expect(t, road_point_covered(&scene, 2, {-1, 0, 5.8}), "wide asymmetric carriageway must be preserved")
    for tri in scene.layers[2] {
        for p in tri { testing.expect(t, p[0] >= -8.001 && p[0] <= 1.501 && p[2] >= -5.001 && p[2] <= 6.001) }
    }
}

@(test)
test_camera_fits_3d_bounds_at_wide_and_portrait_aspects :: proc(t: ^testing.T) {
    low, high := Vec3{-145, 0, -130}, Vec3{145, 75, 130}
    for aspect in ([]f32{1.6, 0.75}) {
        view := fit_bounds_view(low, high, aspect)
        camera := view_camera(view)
        back := (camera.position-camera.target)/view.distance
        right := Vec3{math.cos(view.yaw), 0, -math.sin(view.yaw)}
        up := Vec3{-math.sin(view.pitch)*math.sin(view.yaw), math.cos(view.pitch), -math.sin(view.pitch)*math.cos(view.yaw)}
        for x in ([2]f32{low[0], high[0]}) {
            for y in ([2]f32{low[1], high[1]}) {
                for z in ([2]f32{low[2], high[2]}) {
                    p := Vec3{x, y, z}-view.target
                    depth := view.distance-(p[0]*back[0]+p[1]*back[1]+p[2]*back[2])
                    horizontal := math.abs(p[0]*right[0]+p[2]*right[2])/(depth*math.tan(f32(math.PI/8))*aspect)
                    vertical := math.abs(p[0]*up[0]+p[1]*up[1]+p[2]*up[2])/(depth*math.tan(f32(math.PI/8)))
                    testing.expect(t, depth > 0 && horizontal <= 0.901 && vertical <= 0.821, "all corners inside HUD-safe viewport")
                }
            }
        }
    }
}

@(test)
test_default_showcase_is_populated_and_debug_scene_remains_available :: proc(t: ^testing.T) {
    world: sim.World
    defer sim.destroy(&world)
    testing.expect(t, load_map(&world, DEFAULT_MAP_PATH), "default is a required explicit authored asset, not a demo fallback")
    if len(world.roads) == 0 { return }
    testing.expect(t, len(world.signals) > 0 && len(world.spawns) > 0)
    testing.expect(t, len(world.spawns) == 11, "current authored benchmark has 11 synthetic sources; regenerate stale sidecars")
    turn_source := false
    for source, i in world.spawns {
        road := world.roads[source.road]
        if (world.nodes[road.from].position == sim.Vec2{-135, 0} &&
           world.nodes[road.to].position == sim.Vec2{-45, 0} &&
           world.nodes[source.destination].position == sim.Vec2{-45, 120} &&
           source.kind == .Motorcycle && source.interval == 8) {
            next := world.spawn_runtime[i].next[road.to]
            testing.expect(t, next >= 0 && world.roads[next].to == source.destination, "west-turn benchmark source must actually route through the focused junction")
            turn_source = true
        }
    }
    testing.expect(t, turn_source, "moto-west-turn is explicit synthetic turn coverage, not measured demand")
    buildings := load_building_scene(DEFAULT_MAP_PATH)
    defer destroy_building_scene(&buildings)
    testing.expect(t, buildings.count > 0)
    testing.expect(t, warmup_scene(&world, true, false) == 1200)
    testing.expect(t, math.abs(world.time-60) < 0.001 && len(world.vehicles) > 0, "arrival view must contain simulated road users at 60 seconds")
    sim.destroy(&world); sim.init_demo(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    roads := build_road_scene(&world, offsets)
    defer destroy_road_scene(&roads)
    testing.expect(t, len(roads.layers[2]) > 0 && len(world.signals) == 0 && len(world.spawns) == 0)
}

@(test)
test_shared_junction_polygon_preserves_authored_asphalt_footprint :: proc(t: ^testing.T) {
    // Independent fixed areas from the original mouth/endpoint envelope. The
    // shared fit authority must not grow pavement to hide a body-path failure.
    for fixture in ([]struct { arterial, area: f32 }{{3.5, 104.125}, {5.5, 181.5}}) {
        a := fixture.arterial
        world: sim.World
        sim.init(&world, []sim.Node{{{0, 0}}, {{-120, 0}}, {{120, 0}}, {{0, 120}}, {{0, -120}}},
            []sim.Road{{1, 0, a, 9}, {0, 1, a, 9}, {0, 2, a, 9}, {2, 0, a, 9},
                       {3, 0, 3.5, 9}, {0, 3, 3.5, 9}, {0, 4, 3.5, 9}, {4, 0, 3.5, 9}})
        defer sim.destroy(&world)
        offsets := road_offsets(&world)
        defer delete(offsets)
        scene := build_road_scene(&world, offsets)
        defer destroy_road_scene(&scene)
        area: f32
        for tri in scene.layers[2][len(world.roads)*2:] { area += road_turn(tri[0], tri[1], tri[2])*0.5 }
        testing.expect(t, math.abs(area-fixture.area) < 0.001, "authored junction asphalt envelope must remain unchanged")
    }
}
