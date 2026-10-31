package app

import "core:math"
import "core:path/filepath"
import "core:testing"
import sim "../sim"

@(test)
test_showcase_focus_is_distinct_from_home_overview :: proc(t: ^testing.T) {
    world: sim.World
    defer sim.destroy(&world)
    testing.expect(t, load_map(&world, SHOWCASE_MAP_PATH))
    if len(world.signals) == 0 { return }
    buildings := load_building_scene(SHOWCASE_MAP_PATH)
    defer destroy_building_scene(&buildings)
    west := world.nodes[world.signals[0].node].position
    for aspect in ([]f32{1.6, 0.75}) {
        focus := scene_entry_view(&world, &buildings, .Showcase, aspect)
        home := fit_scene_view(&world, &buildings, aspect)
        testing.expect(t, focus.target == Vec3{west.x, 0, -west.y}, "entry camera must center the west signalized crossing")
        span := 2*focus.distance*math.tan(f32(math.PI/8))*aspect
        testing.expect(t, math.abs(span-150) < 0.01 && focus.extent == 150)
        testing.expect(t, focus.distance < home.distance*0.6 && focus.pitch > home.pitch, "closer/higher gameplay composition, not a whole-map fit")
        testing.expect(t, home.extent > 270 && home.target != focus.target, "Home must preserve full neighborhood overview")
        testing.expect(t, scene_entry_view(&world, &buildings, .Fitted, aspect) == home, "other fixtures retain bounds-fit camera")
        camera := view_camera(focus)
        testing.expect(t, camera.position[1] > 0 && camera.target == focus.target)
    }
}

@(test)
test_showcase_path_and_warmup_match_normal_and_smoke :: proc(t: ^testing.T) {
    absolute, err := filepath.abs(SHOWCASE_MAP_PATH)
    defer delete(absolute)
    testing.expect(t, err == nil && is_showcase_map(absolute) && is_showcase_map("./build/maps/showcase.csv"))
    testing.expect(t, !is_showcase_map(DEFAULT_MAP_PATH) && !is_showcase_map("data/hcm.csv") && !is_showcase_map("/tmp/showcase.csv"))
    testing.expect(t, is_default_map(DEFAULT_MAP_PATH) && is_default_map("./build/maps/intersection.csv"))
    testing.expect(t, scene_profile(SHOWCASE_MAP_PATH) == .Showcase && scene_profile(DEFAULT_MAP_PATH) == .Intersection)
    testing.expect(t, scene_profile("data/hcm.csv") == .Fitted)
    normal, smoke: sim.World
    defer sim.destroy(&normal)
    defer sim.destroy(&smoke)
    testing.expect(t, load_map(&normal, SHOWCASE_MAP_PATH) && load_map(&smoke, SHOWCASE_MAP_PATH))
    testing.expect(t, warmup_scene(&normal, .Showcase, false) == 1200)
    testing.expect(t, warmup_scene(&smoke, .Showcase, true) == 1200)
    expect_same_traffic(t, &normal, &smoke)
    testing.expect(t, math.abs(normal.time-60) < 0.001)
    testing.expect(t, reset_world(&normal) && normal.time == 0)
    testing.expect(t, warmup_scene(&normal, .Fitted, false) == 0 && normal.time == 0)
    testing.expect(t, warmup_scene(&normal, .Fitted, true) == 400 && math.abs(normal.time-20) < 0.001)
    testing.expect(t, reset_world(&normal) && load_map(&normal, DEFAULT_MAP_PATH))
    testing.expect(t, warmup_scene(&normal, .Intersection, false) == 600 && math.abs(normal.time-30) < 0.001)
}

// OpenGL perspective depth in [0,1]; 24-bit quantization is conservative for
// higher-precision buffers. Test projected separation, not arbitrary mesh lifts.
projected_depth :: proc(distance, near, far: f64) -> f64 {
    return far/(far-near)-(far*near/(far-near))/distance
}

@(test)
test_gameplay_clip_range_separates_pavement_without_clipping_scene :: proc(t: ^testing.T) {
    world: sim.World
    defer sim.destroy(&world)
    if !testing.expect(t, load_map(&world, SHOWCASE_MAP_PATH)) { return }
    buildings := load_building_scene(SHOWCASE_MAP_PATH)
    defer destroy_building_scene(&buildings)
    offsets := road_offsets(&world)
    defer delete(offsets)
    roads := build_road_scene(&world, offsets)
    defer destroy_road_scene(&roads)
    entry := scene_entry_view(&world, &buildings, .Showcase)
    old_units := math.abs(projected_depth(f64(entry.distance), 0.05, 4000)-projected_depth(f64(entry.distance)-0.015*f64(math.sin(entry.pitch)), 0.05, 4000))*16777215
    testing.expect(t, old_units < 1, "reproduce default-clip precision loss at the unchanged entry camera")
    close := entry
    close.distance = 8
    for view in ([]View{entry, close, fit_scene_view(&world, &buildings), fit_scene_view(&world, &buildings, 0.75)}) {
        near, far := camera_clip_planes(view)
        testing.expect(t, near > 0 && near < f64(view.distance)*0.1 && far > f64(view.distance))
        camera := view_camera(view)
        back := (camera.position-camera.target)/view.distance
        for tri in roads.layers[2] {
            for point in tri {
                delta := point-camera.position
                depth := -f64(delta[0]*back[0]+delta[1]*back[1]+delta[2]*back[2])
                if depth <= near { continue } // Behind close-camera view, not visible.
                separation := math.abs(projected_depth(depth, near, far)-projected_depth(depth-0.015*f64(back[1]), near, far))*16777215
                if view.distance != 8 {
                    testing.expect(t, depth < far && separation > 4, "entry/Home pavement retains multiple depth units between layers")
                }
            }
        }
        if view.distance != 8 {
            for tri in buildings.triangles {
                for point in ([3]Vec3{tri.a, tri.b, tri.c}) {
                    delta := point-camera.position
                    depth := -f64(delta[0]*back[0]+delta[1]*back[1]+delta[2]*back[2])
                    testing.expect(t, depth > near && depth < far, "entry/Home must retain all actual building depths")
                }
            }
        }
        target_units := math.abs(projected_depth(f64(view.distance), near, far)-projected_depth(f64(view.distance)-0.015*f64(back[1]), near, far))*16777215
        testing.expect(t, target_units > 4, "close, entry and Home target pavement must stay depth-resolvable")
    }
}
