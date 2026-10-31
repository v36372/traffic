package app

import "core:fmt"
import "core:math"
import "core:testing"
import mapdata "../mapdata"
import sim "../sim"

check_roof :: proc(t: ^testing.T, points: []sim.Vec2) {
    roof, ok := triangulate_roof(points)
    defer delete(roof)
    testing.expect(t, ok)
    if !ok { return }
    expected, actual: f64
    for p, i in points {
        q := points[(i+1)%len(points)]
        expected += f64(p.x)*f64(q.y)-f64(q.x)*f64(p.y)
    }
    for tri in roof {
        a, b, c := points[tri[0]], points[tri[1]], points[tri[2]]
        cross := turn_2d(a, b, c)
        testing.expect(t, cross > 0, "roof triangles must be CCW in east/north")
        ab, ac := building_vertex(b, 0)-building_vertex(a, 0), building_vertex(c, 0)-building_vertex(a, 0)
        testing.expect(t, ab[2]*ac[0]-ab[0]*ac[2] > 0, "roof must face +Y after projection")
        actual += cross
    }
    testing.expect(t, math.abs(actual-math.abs(expected)) < 0.001, "triangles must cover polygon area exactly")
}

@(test)
test_concave_roof_winding_and_collinear_vertices :: proc(t: ^testing.T) {
    // L footprint: a triangle fan would cross the missing upper-right quadrant.
    points := []sim.Vec2{{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}}
    check_roof(t, points)
    roof, ok := triangulate_roof(points)
    defer delete(roof)
    testing.expect(t, ok && len(roof) == 4)
    for tri in roof {
        a, b, c := points[tri[0]], points[tri[1]], points[tri[2]]
        x, y := (a.x+b.x+c.x)/3, (a.y+b.y+c.y)/3
        testing.expect(t, x <= 2 || y <= 2, "concave courtyard must remain empty")
    }
    // Input reversal still produces upward roofs.
    reverse := []sim.Vec2{{0, 6}, {2, 6}, {2, 2}, {6, 2}, {6, 0}, {0, 0}}
    check_roof(t, reverse)
    check_roof(t, []sim.Vec2{{0, 0}, {2, 0}, {4, 0}, {4, 4}, {0, 4}})
    invalid, valid := triangulate_roof([]sim.Vec2{{0, 0}, {1, 0}, {2, 0}})
    defer delete(invalid)
    testing.expect(t, !valid && len(invalid) == 0)
}

@(test)
test_actual_building_count_roofs_heights_and_cleanup :: proc(t: ^testing.T) {
    buildings, ok := mapdata.load_buildings("data/hcm-buildings.csv")
    testing.expect(t, ok)
    if !ok { return }
    defer mapdata.destroy_buildings(buildings)
    // 129 source building ways minus 3 roofs, 2 construction and unsupported geometry.
    testing.expect(t, len(buildings) == 123)
    for b in buildings { check_roof(t, b.footprint) }
    for _ in 0..<3 {
        scene := build_building_scene(buildings[:])
        testing.expect(t, scene.count == len(buildings), "no actual footprint may silently disappear")
        top: f32
        for tri in scene.triangles { top = math.max(top, math.max(tri.a[1], math.max(tri.b[1], tri.c[1]))) }
        testing.expect(t, top == 193, "source-tagged tower height must not be flattened")
        fmt.printf("Building cache: %d footprints, %d triangles, tallest %.0fm\n", scene.count, len(scene.triangles), top)
        destroy_building_scene(&scene)
        destroy_building_scene(&scene)
        testing.expect(t, scene.count == 0 && len(scene.triangles) == 0 && cap(scene.triangles) == 0)
    }
}

@(test)
test_setbacks_remain_inside_convex_source_and_preserve_height :: proc(t: ^testing.T) {
    outline := []sim.Vec2{{0, 0}, {20, 0}, {20, 40}, {0, 40}}
    buildings := []mapdata.Building{{footprint = outline, height = 170, kind = .Office}}
    scene := build_building_scene(buildings)
    defer destroy_building_scene(&scene)
    testing.expect(t, scene.count == 1 && len(scene.triangles) == 20)
    inset, full, top := false, false, false
    for tri in scene.triangles {
        for p in ([3]Vec3{tri.a, tri.b, tri.c}) {
            testing.expect(t, p[0] >= 0 && p[0] <= 20 && p[2] >= -40 && p[2] <= 0)
            if p[1] > 12 {
                testing.expect(t, p[0] > 0 && p[0] < 20 && p[2] > -40 && p[2] < 0)
                inset = true
            }
            if p[0] == 0 { full = true }
            if p[1] == 170 { top = true }
        }
    }
    testing.expect(t, inset && full && top)
    for kind in ([]mapdata.Building_Kind{.Lowrise, .Apartment, .Landmark}) {
        buildings[0].kind = kind
        flat := build_building_scene(buildings)
        testing.expect(t, len(flat.triangles) == 10, "non-towers retain a simple flat silhouette")
        destroy_building_scene(&flat)
    }
}

@(test)
test_optional_sidecar_and_tall_scene_fit :: proc(t: ^testing.T) {
    for pair in ([][2]string{{"data/hcm.csv", "data/hcm-buildings.csv"}, {"/tmp/map.v2.csv", "/tmp/map.v2-buildings.csv"}, {"map", "map-buildings.csv"}}) {
        path := building_sidecar(pair[0])
        testing.expect(t, path == pair[1])
        delete(path)
    }
    missing := load_building_scene("data/not-present.csv")
    defer destroy_building_scene(&missing)
    testing.expect(t, missing.count == 0)
    world: sim.World
    sim.init_demo(&world)
    defer sim.destroy(&world)
    scene := build_building_scene([]mapdata.Building{{footprint = []sim.Vec2{{0, 0}, {20, 0}, {20, 20}, {0, 20}}, height = 193, kind = .Office}})
    defer destroy_building_scene(&scene)
    view := fit_scene_view(&world, &scene)
    testing.expect(t, view.target[1] == 96.5 && view.distance > 193)
    // Traffic reset does not rebuild or replace the independently owned cache.
    data, count := raw_data(scene.triangles), len(scene.triangles)
    sim.destroy(&world); sim.init_demo(&world)
    testing.expect(t, raw_data(scene.triangles) == data && len(scene.triangles) == count)
    // Demo transition destroys the map cache, including its capacity.
    destroy_building_scene(&scene)
    testing.expect(t, scene.count == 0 && cap(scene.triangles) == 0)
    testing.expect(t, fit_scene_view(&world, &scene) == fit_view(&world))
}

// Independent SAT oracle on rendered triangles, not centerline/setback formulas.
// Building AABBs conservatively contain even concave footprints and their insets.
triangle_overlaps_building_bounds :: proc(tri: [3]Vec3, low, high: Vec3) -> bool {
    center, half := (low+high)*0.5, (high-low)*0.5
    axes := [5]Vec3{{1, 0, 0}, {0, 0, 1}, {}, {}, {}}
    for p, i in tri {
        edge := tri[(i+1)%3]-p
        axes[i+2] = {-edge[2], 0, edge[0]}
    }
    for axis in axes {
        a := tri[0][0]*axis[0]+tri[0][2]*axis[2]
        first, last := a, a
        for i in 1..<3 {
            projection := tri[i][0]*axis[0]+tri[i][2]*axis[2]
            first = math.min(first, projection); last = math.max(last, projection)
        }
        middle := center[0]*axis[0]+center[2]*axis[2]
        radius := half[0]*math.abs(axis[0])+half[2]*math.abs(axis[2])
        if last <= middle-radius+0.0001 || first >= middle+radius-0.0001 { return false }
    }
    return true
}

@(test)
test_authored_showcase_buildings_clear_actual_pavement_and_poles_stay_on_sidewalks :: proc(t: ^testing.T) {
    // A triangle can cross a footprint without any of its own vertices inside.
    testing.expect(t, triangle_overlaps_building_bounds([3]Vec3{{-3, 0, 0}, {3, 0, 0}, {0, 0, 3}}, {-1, 0, -1}, {1, 0, 1}))
    testing.expect(t, !triangle_overlaps_building_bounds([3]Vec3{{3, 0, 3}, {4, 0, 3}, {3, 0, 4}}, {-1, 0, -1}, {1, 0, 1}))
    world: sim.World
    defer sim.destroy(&world)
    if !testing.expect(t, load_map(&world, SHOWCASE_MAP_PATH)) { return }
    central_edges := 0
    for road in world.roads {
        a, b := world.nodes[road.from].position, world.nodes[road.to].position
        if a.y == 0 && b.y == 0 {
            central_edges += 1
            testing.expect(t, road.width == 5.5, "stale showcase CSV: regenerate maps; authored central carriageways must be 5.5m")
        } else {
            testing.expect(t, road.width == 3.5, "only the authored central arterial is widened; side streets stay 7m total")
        }
    }
    testing.expect(t, central_edges == 6, "three authored central links must produce six directed carriageways")
    path := building_sidecar(SHOWCASE_MAP_PATH)
    defer delete(path)
    buildings, ok := mapdata.load_buildings(path)
    defer mapdata.destroy_buildings(buildings)
    if !testing.expect(t, ok && len(buildings) > 0) { return }
    offsets := road_offsets(&world)
    defer delete(offsets)
    roads := build_road_scene(&world, offsets)
    defer destroy_road_scene(&roads)
    testing.expect(t, len(roads.layers[2]) > len(world.roads)*2, "oracle must include actual junction hulls, not just road strips")
    for building, i in buildings {
        low := building_vertex(building.footprint[0], 0)
        high := low
        for p in building.footprint {
            point := building_vertex(p, 0)
            low[0] = math.min(low[0], point[0]); low[2] = math.min(low[2], point[2])
            high[0] = math.max(high[0], point[0]); high[2] = math.max(high[2], point[2])
        }
        for layer in ([]int{0, 2}) {
            for tri in roads.layers[layer] {
                testing.expect(t, !triangle_overlaps_building_bounds(tri, low, high), fmt.tprintf("building %d bounds overlap rendered layer %d (0 sidewalk, 2 asphalt)", i, layer))
            }
        }
    }
    approaches := signal_approaches(&world, offsets)
    defer delete(approaches)
    testing.expect(t, len(approaches) > 0)
    for approach in approaches {
        for x in ([]f32{-0.16, 0.16}) {
            for z in ([]f32{-0.16, 0.16}) {
                corner := approach.pole+Vec3{x, 0, z}
                testing.expect(t, !road_point_covered(&roads, 2, corner), "showcase signal base must clear actual asphalt/hulls")
                testing.expect(t, road_point_covered(&roads, 0, corner), "showcase signal base must stay on actual sidewalk")
            }
        }
    }
}
