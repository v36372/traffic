package app

import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_crossing_paint_full_span_stop_order_and_actual_curb_landings :: proc(t: ^testing.T) {
    for geometry in ([]struct { skew: bool, widths: [4]f32 }{
        {false, {5.5, 5.5, 3.5, 3.5}},
        {false, {3, 6, 9, 4}},
        {true, {4, 3, 8, 2}},
    }) {
        arm := sim.Vec2{0, 60}
        if geometry.skew { arm = {42, 42} }
        w := geometry.widths
        world: sim.World
        sim.init(&world, []sim.Node{{{0, 0}}, {{-60, 0}}, {{60, 0}}, {arm}, {{-arm.x, -arm.y}}},
            []sim.Road{{1, 0, w[0], 9}, {0, 1, w[1], 9}, {0, 2, w[1], 9}, {2, 0, w[0], 9},
                       {3, 0, w[2], 9}, {0, 3, w[3], 9}, {0, 4, w[3], 9}, {4, 0, w[2], 9}})
        defer sim.destroy(&world)
        testing.expect(t, sim.configure(&world, []sim.Signal_Plan{{0, 100, 1, 1, 0}}, nil))
        offsets := road_offsets(&world)
        defer delete(offsets)
        scene := build_road_scene(&world, offsets)
        defer destroy_road_scene(&scene)
        testing.expect(t, len(scene.crosswalks) > 0 && len(scene.landings) == 16)
        for tri in scene.crosswalks {
            testing.expect(t, road_turn(tri[0], tri[1], tri[2]) > 0)
            for point in ([4]Vec3{tri[0], tri[1], tri[2], (tri[0]+tri[1]+tri[2])/3}) {
                testing.expect(t, math.abs(point[1]-CROSSING_PAINT_HEIGHT) < 0.000001, "paint height must match the marking plane (including averaged sample points)")
                testing.expect(t, road_point_covered(&scene, 2, point), "paint must stay on actual asphalt")
                for hull in scene.layers[2][len(world.roads)*2:] {
                    testing.expect(t, !road_triangle_covers(hull, point), "pedestrian band must be upstream of actual junction hull")
                }
            }
        }
        for tri in scene.landings {
            testing.expect(t, road_turn(tri[0], tri[1], tri[2]) > 0)
            for point in ([4]Vec3{tri[0], tri[1], tri[2], (tri[0]+tri[1]+tri[2])/3}) {
                // The inner landing edge touches the carriageway; sample a small
                // interior step to avoid classifying that shared boundary as spill.
                center := (tri[0]+tri[1]+tri[2])/3
                interior := point*0.99+center*0.01
                testing.expect(t, road_point_covered(&scene, 0, point), "landing must join actual existing sidewalk")
                testing.expect(t, !road_point_covered(&scene, 2, interior), "landing must not add/overpaint asphalt")
            }
        }
        approaches := signal_approaches(&world, offsets)
        defer delete(approaches)
        for approach in approaches {
            start, forward, left, _ := road_frame(&world, world.roads[approach.road])
            near, far, center_lateral, width, ok := sim.crossing_band(&world, approach.road)
            testing.expect(t, ok && math.abs(far-near-4) < 0.001)
            if !ok { continue }
            // Derive curb positions from BOTH actual strips, not the span helper.
            minimum, maximum := offsets[approach.road]-approach.stop_width*0.5, offsets[approach.road]+approach.stop_width*0.5
            r := world.roads[approach.road]
            for other, j in world.roads {
                if other.from == r.to && other.to == r.from {
                    minimum = math.min(minimum, -offsets[j]-other.width*0.5)
                    maximum = math.max(maximum, -offsets[j]+other.width*0.5)
                }
            }
            testing.expect(t, math.abs(center_lateral-width*0.5-minimum) < 0.001 && math.abs(center_lateral+width*0.5-maximum) < 0.001, "full crossing span must include unequal reciprocal carriageways without double offset")
            bar := approach.stop_center+forward*0.125
            bar_distance := (bar[0]-start[0])*forward[0]+(bar[2]-start[2])*forward[2]
            testing.expect(t, near-bar_distance >= 0.874, "entire visual bar must be upstream of the crossing, never painted over it")
            found := false
            for tri in scene.crosswalks {
                center := (tri[0]+tri[1]+tri[2])/3
                delta := center-start
                along := delta[0]*forward[0]+delta[2]*forward[2]
                lateral := delta[0]*left[0]+delta[2]*left[2]
                if along < near || along > far || math.abs(lateral-center_lateral) > width*0.5 { continue }
                found = true
                for point in tri {
                    d := point-start
                    testing.expect(t, d[0]*forward[0]+d[2]*forward[2] > bar_distance, "actual white stripe must be downstream of the actual stop bar")
                    for other, j in world.roads {
                        _, other_forward, _, _ := road_frame(&world, other)
                        cross := forward[0]*other_forward[2]-forward[2]*other_forward[0]
                        if math.abs(cross) < 0.0001 { continue }
                        for rectangle in scene.layers[2][j*2:j*2+2] {
                            testing.expect(t, !road_triangle_covers(rectangle, point), "white stripes must remain upstream of ALL crossing-road strips, including skew arms")
                        }
                    }
                }
            }
            testing.expect(t, found, "each supported controlled approach needs visible crossing paint")
        }
        destroy_road_scene(&scene)
        testing.expect(t, cap(scene.crosswalks) == 0 && cap(scene.landings) == 0)
    }
}

@(test)
test_crossing_paint_never_invents_controls_or_short_layout_fallback :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{-3, 0}}, {{0, 3}}}, []sim.Road{{1, 0, 3.5, 9}, {2, 0, 3.5, 9}})
    defer sim.destroy(&world)
    offsets := road_offsets(&world)
    defer delete(offsets)
    scene := build_road_scene(&world, offsets)
    testing.expect(t, len(scene.crosswalks) == 0 && len(scene.landings) == 0)
    destroy_road_scene(&scene)
    // Deliberately invalid short controlled geometry: isolate paint omission;
    // actual world configuration rejects an unusable controlled stop position.
    append(&world.signals, sim.Signal_Plan{node = 0})
    scene = build_road_scene(&world, offsets)
    defer destroy_road_scene(&scene)
    testing.expect(t, len(scene.crosswalks) == 0 && len(scene.landings) == 0)
}
