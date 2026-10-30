package app

import "core:fmt"
import "core:math"
import "core:os"
import "core:testing"
import sim "../sim"

@(test)
test_cli_capture_validation :: proc(t: ^testing.T) {
    defaults, ok := parse_options(nil)
    testing.expect(t, ok && defaults.demand_scale == 1 && defaults.warmup == -1 && !defaults.smoke)
    testing.expect(t, defaults.map_path == DEFAULT_MAP_PATH && defaults.sequence_dir == "")
    for flag in ([]string{"--map", "--screenshot", "--capture-sequence", "--demand-scale", "--warmup"}) {
        _, missing := parse_options([]string{flag})
        _, empty := parse_options([]string{flag, ""})
        _, next_flag := parse_options([]string{flag, "--smoke-test"})
        testing.expect(t, !missing && !empty && !next_flag)
    }
    for value in ([]string{"nan", "inf", "-inf", "abc", "1x", "1e999"}) {
        for flag in ([]string{"--demand-scale", "--warmup"}) {
            _, accepted := parse_options([]string{flag, value})
            testing.expect(t, !accepted)
        }
    }
    for value in ([]string{"0", "-1", "20.01", "1e-99"}) {
        _, accepted := parse_options([]string{"--demand-scale", value})
        testing.expect(t, !accepted)
    }
    for value in ([]string{"-0.01", "3600.01"}) {
        _, accepted := parse_options([]string{"--warmup", value})
        testing.expect(t, !accepted)
    }
    _, unknown := parse_options([]string{"--unknown"})
    testing.expect(t, !unknown)
    options, valid := parse_options([]string{"--map", "fixture.csv", "--demand-scale", "0.25", "--warmup", "12.5", "--capture-sequence", "frames", "--screenshot", "single.png"})
    testing.expect(t, valid && options.smoke && options.demand_scale == 0.25 && options.warmup == 12.5)
    testing.expect(t, options.map_path == "fixture.csv" && options.sequence_dir == "frames" && options.screenshot == "single.png")
    boundary, boundary_ok := parse_options([]string{"--demand-scale", "20", "--warmup", "3600"})
    testing.expect(t, boundary_ok && boundary.demand_scale == 20 && boundary.warmup == 3600)
}

@(test)
test_capture_directory_probe :: proc(t: ^testing.T) {
    dir, err := os.make_directory_temp("", "traffic-capture-test-*", context.allocator)
    testing.expect(t, err == nil)
    if err != nil { return }
    defer delete(dir)
    defer os.remove(dir)
    testing.expect(t, sequence_directory_ready(dir))
    missing := fmt.aprintf("%s/missing", dir)
    defer delete(missing)
    testing.expect(t, !sequence_directory_ready(missing) && !sequence_directory_ready(""))
    file := fmt.aprintf("%s/not-a-directory", dir)
    defer delete(file)
    defer os.remove(file)
    testing.expect(t, os.write_entire_file(file, "test") == nil)
    testing.expect(t, !sequence_directory_ready(file))
    when ODIN_OS != .Windows {
        testing.expect(t, os.change_mode(dir, {.Read_User, .Execute_User}) == nil)
        // This suite runs as an unprivileged user; root bypasses DAC permissions.
        if os.get_euid() != 0 { testing.expect(t, !sequence_directory_ready(dir)) }
        testing.expect(t, os.change_mode(dir, {.Read_User, .Write_User, .Execute_User}) == nil)
    }
}

@(test)
test_capture_cadence_and_seeded_clock_equivalence :: proc(t: ^testing.T) {
    smoke, sequence, reference: sim.World
    sim.init_demo(&smoke, 42)
    sim.init_demo(&sequence, 42)
    sim.init_demo(&reference, 42)
    defer sim.destroy(&smoke)
    defer sim.destroy(&sequence)
    defer sim.destroy(&reference)
    testing.expect(t, warmup_scene(&smoke, false, true, 1.25) == 25)
    testing.expect(t, warmup_scene(&sequence, false, true, 1.25) == 25)
    testing.expect(t, warmup_scene(&reference, false, false, 1.25) == 25)
    captured := 0
    for frame in 1..=SEQUENCE_FRAMES {
        advance_capture_frame(&sequence, frame)
        if frame <= 90 { advance_capture_frame(&smoke, frame) }
        if frame == 90 {
            expect_same_traffic(t, &smoke, &sequence)
            testing.expect(t, math.abs(smoke.time-2.75) < 0.0001)
        }
        if capture_due(frame) { captured += 1; testing.expect(t, frame == captured*6) }
    }
    for _ in 0..<200 { sim.step(&reference, f32(TICK)) }
    expect_same_traffic(t, &sequence, &reference)
    testing.expect(t, captured == 100 && !capture_due(0) && !capture_due(-6) && !capture_due(606))
    testing.expect(t, math.abs(sequence.time-11.25) < 0.0001)
    // Explicit warmup overrides both defaults and rounds to the common tick grid.
    testing.expect(t, reset_world(&smoke))
    testing.expect(t, warmup_scene(&smoke, true, true, 0) == 0 && smoke.time == 0)
    testing.expect(t, warmup_scene(&smoke, true, false, 0.126) == 3)
}

@(test)
test_scaled_sources_are_transactional_and_reset_keeps_intervals :: proc(t: ^testing.T) {
    dir, err := os.make_directory_temp("", "traffic-scale-test-*", context.allocator)
    testing.expect(t, err == nil)
    if err != nil { return }
    defer delete(dir)
    defer os.remove(dir)
    path := fmt.aprintf("%s/map.csv", dir)
    defer delete(path)
    defer os.remove(path)
    sidecar := objects_sidecar(path)
    defer delete(sidecar)
    defer os.remove(sidecar)
    testing.expect(t, os.write_entire_file(path, "HCM_MAP,1\nN,0,0\nN,100,0\nN,200,0\nE,0,1,7,9\nE,1,2,7,9\n") == nil)
    testing.expect(t, os.write_entire_file(sidecar, "TRAFFIC_OBJECTS,1\nP,0,2,0.8,0\n") == nil)
    scaled, manual, quiet: sim.World
    defer sim.destroy(&scaled)
    defer sim.destroy(&manual)
    defer sim.destroy(&quiet)
    testing.expect(t, load_map(&scaled, path, 2) && scaled.spawns[0].interval == 0.4)
    testing.expect(t, load_map(&manual, path) && sim.configure(&manual, nil, []sim.Spawn_Point{{0, 2, 0.4, .Motorcycle}}))
    testing.expect(t, load_map(&quiet, path, 0.25) && quiet.spawns[0].interval == 3.2)
    for node, i in scaled.nodes { testing.expect(t, node == quiet.nodes[i]) }
    for road, i in scaled.roads { testing.expect(t, road == quiet.roads[i]) }
    buildings: Building_Scene
    testing.expect(t, scene_entry_view(&scaled, &buildings, false) == scene_entry_view(&quiet, &buildings, false))
    for _ in 0..<200 { sim.step(&scaled, f32(TICK)); sim.step(&manual, f32(TICK)); sim.step(&quiet, f32(TICK)) }
    expect_same_traffic(t, &scaled, &manual)
    testing.expect(t, scaled.requested > quiet.requested)
    testing.expect(t, load_map(&quiet, path, 16) && quiet.spawns[0].interval == 0.05)
    testing.expect(t, load_map(&quiet, path, f32(0.8/86400.0)) && quiet.spawns[0].interval == 86400)
    // Out-of-bounds scaled intervals reject the entire load, not clamp silently.
    clock, rng, requested := scaled.time, scaled.rng, scaled.requested
    for scale in ([]f32{20, 0.000001, 0, -1, 21}) {
        testing.expect(t, !load_map(&scaled, path, scale))
        testing.expect(t, scaled.time == clock && scaled.rng == rng && scaled.requested == requested && scaled.spawns[0].interval == 0.4)
    }
    testing.expect(t, reset_world(&scaled) && scaled.spawns[0].interval == 0.4 && scaled.time == 0)
    testing.expect(t, reset_world(&manual))
    for _ in 0..<200 { sim.step(&scaled, f32(TICK)); sim.step(&manual, f32(TICK)) }
    expect_same_traffic(t, &scaled, &manual)
    testing.expect(t, os.remove(sidecar) == nil)
    testing.expect(t, !load_map(&scaled, path, 2) && scaled.time > 0)
    testing.expect(t, load_map(&scaled, path) && len(scaled.spawns) == 0)
}

@(test)
test_renderer_uses_sim_pose_without_double_offset :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{100, 0}}}, []sim.Road{{0, 1, 6, 9}, {1, 0, 6, 9}})
    defer sim.destroy(&world)
    for road in 0..<2 {
        vehicle := sim.Vehicle{road = road, distance = 20, lateral = 0.8, kind = .Motorcycle}
        point, heading := sim.vehicle_pose(&world, vehicle)
        position, forward := vehicle_render_pose(&world, vehicle)
        testing.expect(t, position == Vec3{point.x, 0, -point.y})
        testing.expect(t, forward == Vec3{heading.x, 0, -heading.y})
        testing.expect(t, math.abs(position[2]) > 1)
    }
    shirt, helmet, body := rider_colors(0)
    other_shirt, other_helmet, other_body := rider_colors(1)
    testing.expect(t, shirt != other_shirt && helmet != other_helmet && body != other_body)
    testing.expect(t, shirt.a == 255 && helmet.a == 255 && body.a == 255)
    // IDs stay u64 until after modulo, including values beyond signed int range.
    high_shirt, high_helmet, high_body := rider_colors(0xffff_ffff_ffff_ffff)
    testing.expect(t, high_shirt.a == 255 && high_helmet.a == 255 && high_body.a == 255)
}

@(test)
test_large_vehicle_body_colors_vary_by_stable_id :: proc(t: ^testing.T) {
    for kind in ([]sim.Vehicle_Kind{.Car, .Bus, .Truck}) {
        first := vehicle_color(kind, 1)
        for id: u64 = 2; id <= 6; id += 1 {
            color := vehicle_color(kind, id)
            testing.expect(t, color != first && color.a == 255)
            testing.expect(t, color == vehicle_color(kind, id))
        }
        testing.expect(t, vehicle_color(kind, 0xffff_ffff_ffff_ffff).a == 255)
    }
}
