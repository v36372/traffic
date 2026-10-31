package app

import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_signals_clear_all_junction_pavement_and_vehicle_front_meets_bar :: proc(t: ^testing.T) {
    // All fixtures have exactly one shared junction. The last case has skew
    // crossing strips, which a half-crossroad-width-only setback would miss.
    for geometry in ([]struct { skew: bool, widths: [4]f32 }{
        {false, {4, 4, 4, 4}},
        {false, {3, 6, 9, 4}},
        {true, {4, 3, 8, 2}},
    }) {
        arm := sim.Vec2{0, 60}
        if geometry.skew { arm = {42, 42} }
        nodes := []sim.Node{{{0, 0}}, {{-60, 0}}, {{60, 0}}, {arm}, {{-arm.x, -arm.y}}}
        w := geometry.widths
        input := []sim.Road{
            {1, 0, w[0], 10}, {0, 1, w[1], 10},
            {0, 2, w[1], 10}, {2, 0, w[0], 10},
            {3, 0, w[2], 10}, {0, 3, w[3], 10},
            {0, 4, w[3], 10}, {4, 0, w[2], 10},
        }
        world: sim.World
        sim.init(&world, nodes, input)
        defer sim.destroy(&world)
        testing.expect(t, sim.configure(&world, []sim.Signal_Plan{{0, 100, 1, 1, 0, false}}, nil))
        offsets := road_offsets(&world)
        defer delete(offsets)
        pavement := build_road_scene(&world, offsets)
        defer destroy_road_scene(&pavement)
        testing.expect(t, len(pavement.junction_nodes) == 1)
        approaches := signal_approaches(&world, offsets)
        defer delete(approaches)
        testing.expect(t, len(approaches) == 4)
        for approach in approaches {
            start, forward, _, _ := road_frame(&world, world.roads[approach.road])
            // Test actual rendered triangles independently of the shared stop helper.
            testing.expect(t, !road_point_covered(&pavement, 2, approach.pole), "signal pole must not be on any crossing/approach/junction asphalt")
            // The entire 0.32m pole base must clear pavement, not just its center.
            for x in ([]f32{-0.16, 0.16}) {
                for z in ([]f32{-0.16, 0.16}) {
                    testing.expect(t, !road_point_covered(&pavement, 2, approach.pole+Vec3{x, 0, z}), "signal base must clear ALL asphalt")
                }
            }
            bar_end := approach.stop_center+forward*0.125
            bar_distance := (bar_end[0]-start[0])*forward[0]+(bar_end[2]-start[2])*forward[2]
            // Check the two bar endpoints and center against the actual crossing
            // road rectangles too: a skew strip can extend beyond the local hull.
            for other, j in world.roads {
                _, other_forward, _, _ := road_frame(&world, other)
                cross := forward[0]*other_forward[2]-forward[2]*other_forward[0]
                if math.abs(cross) < 0.0001 { continue }
                for side in ([]f32{-approach.stop_width*0.5, 0, approach.stop_width*0.5}) {
                    p := bar_end+approach.left*side
                    for tri in pavement.layers[2][j*2:j*2+2] {
                        testing.expect(t, !road_triangle_covers(tri, p), "stop bar must not overlap ANY crossing strip, including skew approaches")
                    }
                }
            }
            // Straight strips are cached first (2 triangles per road), followed by
            // the sole junction hull. Its most-upstream vertex bounds the crossing.
            for tri in pavement.layers[2][len(input)*2:] {
                for p in tri {
                    projection := (p[0]-start[0])*forward[0]+(p[2]-start[2])*forward[2]
                    testing.expect(t, bar_distance < projection, "entire stop bar must be upstream of the rendered junction hull")
                }
            }
        }
        if len(approaches) == 0 { continue }
        west := approaches[0]
        start, forward, _, _ := road_frame(&world, world.roads[0])
        bar := (west.stop_center[0]-start[0])*forward[0]+(west.stop_center[2]-start[2])*forward[2]
        world.time = 103 // West approach is red; east approach owns the green.
        testing.expect(t, sim.signal_state(&world, 0) == .Red)
        append(&world.vehicles, sim.Vehicle{road = 0, distance = bar-sim.kind_length(.Car)*0.5-6, speed = 5, kind = .Car, spawn_index = -1})
        for _ in 0..<200 {
            world.spawn_accumulator = -100 // Isolate the manually placed test vehicle.
            sim.step(&world, f32(TICK))
            front := world.vehicles[0].distance+sim.kind_length(.Car)*0.5
            testing.expect(t, front <= bar+0.001, "red vehicle front must never cross the actual visual bar")
        }
        testing.expect(t, math.abs(world.vehicles[0].distance+sim.kind_length(.Car)*0.5-bar) < 0.01, "vehicle must stop AT the visual bar, not at node center")
        testing.expect(t, world.vehicles[0].speed == 0)
    }
}

@(test)
test_original_signalized_fixture_poles_are_on_approach_sidewalks :: proc(t: ^testing.T) {
    world: sim.World
    defer sim.destroy(&world)
    testing.expect(t, load_map(&world, "build/maps/signalized.csv"))
    offsets := road_offsets(&world)
    defer delete(offsets)
    pavement := build_road_scene(&world, offsets)
    defer destroy_road_scene(&pavement)
    approaches := signal_approaches(&world, offsets)
    defer delete(approaches)
    testing.expect(t, len(approaches) == 2)
    if len(approaches) != 2 { return }
    testing.expect(t, approaches[0].pole[0] <= -4.9 && approaches[0].pole[2] > 2, "west light belongs before the crossing on its south sidewalk")
    testing.expect(t, approaches[1].pole[2] >= 4.9 && approaches[1].pole[0] > 2, "south light belongs before the crossing on its east sidewalk")
    for approach in approaches {
        testing.expect(t, !road_point_covered(&pavement, 2, approach.pole), "original fixture must not place either pole in the cross")
        testing.expect(t, road_point_covered(&pavement, 0, approach.pole), "pole should remain on the actual sidewalk band")
    }
}
