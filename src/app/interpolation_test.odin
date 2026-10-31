package app

import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_vehicle_motion_updates_between_ticks_without_changing_simulation :: proc(t: ^testing.T) {
    for fps in ([]int{60, 144, 360}) {
        for speed in ([]f64{0.25, 1, 8}) {
            world, reference: sim.World
            sim.init(&world, []sim.Node{{{0, 0}}, {{1000, 0}}}, []sim.Road{{0, 1, 7, 9}})
            sim.init(&reference, world.nodes[:], world.roads[:])
            defer sim.destroy(&world)
            defer sim.destroy(&reference)
            testing.expect(t, sim.configure(&world, nil, []sim.Spawn_Point{{0, 1, 86400, .Car}}))
            testing.expect(t, sim.configure(&reference, nil, world.spawns[:]))
            vehicle := sim.Vehicle{road = 0, distance = 10, speed = 6, kind = .Car, id = 1}
            append(&world.vehicles, vehicle)
            append(&reference.vehicles, vehicle)
            world.spawned, world.requested = 1, 1
            reference.spawned, reference.requested = 1, 1
            previous := make(Render_Poses)
            defer delete(previous)
            accumulator: f64
            last: Vec3
            ready := false
            moving_frames, ticks := 0, 0
            for frame in 1..=fps {
                accumulator += speed/f64(fps)
                for accumulator >= TICK {
                    remember_vehicle_poses(&previous, &world)
                    sim.step(&world, f32(TICK))
                    sim.step(&reference, f32(TICK))
                    accumulator -= TICK
                    ticks += 1
                }
                alpha := f32(accumulator/TICK)
                p, f := interpolated_vehicle_pose(&world, world.vehicles[0], previous, alpha)
                current, _ := vehicle_render_pose(&world, world.vehicles[0])
                old, found := previous[1]
                testing.expect(t, p[0] <= current[0]+0.00001 && p[0] >= 10, "never extrapolate past simulated movement")
                if found {
                    testing.expect(t, p[0] >= old[0][0]-0.00001)
                    if ready {
                        testing.expect(t, p[0] > last[0], "motion must advance on every rendered frame, not just 20Hz ticks")
                        moving_frames += 1
                    }
                }
                testing.expect(t, f == Vec3{1, 0, 0} || math.abs(f[0]-1) < 0.000001)
                // Pausing keeps this phase: repeated renders cannot move a vehicle
                // or mutate physics/RNG. Resuming can use the same remainder.
                frozen, _ := interpolated_vehicle_pose(&world, world.vehicles[0], previous, alpha)
                testing.expect(t, frozen == p && world.vehicles[0] == reference.vehicles[0] && world.rng == reference.rng)
                last, ready = p, found
            }
            if fps == 60 && speed == 1 { testing.expect(t, moving_frames > ticks*2) }
        }
    }
}

@(test)
test_interpolation_uses_trip_ids_and_clears_reset_history :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{100, 0}}}, []sim.Road{{0, 1, 7, 9}})
    defer sim.destroy(&world)
    append(&world.vehicles, sim.Vehicle{id = 11, road = 0, distance = 10}, sim.Vehicle{id = 12, road = 0, distance = 20})
    previous := make(Render_Poses)
    defer delete(previous)
    remember_vehicle_poses(&previous, &world)
    ordered_remove(&world.vehicles, 0)
    world.vehicles[0].distance = 22
    append(&world.vehicles, sim.Vehicle{id = 13, road = 0, distance = 30})
    p, _ := interpolated_vehicle_pose(&world, world.vehicles[0], previous, 0.5)
    testing.expect(t, math.abs(p[0]-21) < 0.00001, "removal/reindexing must not blend different trips")
    p, _ = interpolated_vehicle_pose(&world, world.vehicles[1], previous, 0)
    testing.expect(t, p[0] == 30, "newly spawned trips have no previous pose")
    remember_vehicle_poses(&previous, &world)
    testing.expect(t, len(previous) == 2 && 11 not_in previous, "history retains one tick, not every historical ID")
    clear(&previous) // Same history reset used by R and map/debug changes.
    world.vehicles[0].distance = 80
    p, _ = interpolated_vehicle_pose(&world, world.vehicles[0], previous, 0.25)
    testing.expect(t, p[0] == 80, "reused IDs after reset must not interpolate from the previous world")
}

@(test)
test_heading_interpolation_crosses_pi_seam_and_clamps_endpoints :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{0.1, 100}}, {{-0.1, 100}}}, []sim.Road{{0, 1, 7, 9}, {0, 2, 7, 9}})
    defer sim.destroy(&world)
    append(&world.vehicles, sim.Vehicle{id = 1, road = 0, distance = 10})
    previous := make(Render_Poses)
    defer delete(previous)
    remember_vehicle_poses(&previous, &world)
    world.vehicles[0].road = 1
    current_p, current_f := vehicle_render_pose(&world, world.vehicles[0])
    for alpha in ([]f32{0.25, 0.5, 0.75}) {
        _, f := interpolated_vehicle_pose(&world, world.vehicles[0], previous, alpha)
        testing.expect(t, f[2] < -0.999 && math.abs(f[0]*f[0]+f[2]*f[2]-1) < 0.000001, "unit heading must take shortest rotation, not spin through north")
    }
    p, f := interpolated_vehicle_pose(&world, world.vehicles[0], previous, -1)
    testing.expect(t, p == previous[1][0] && f == previous[1][1])
    p, f = interpolated_vehicle_pose(&world, world.vehicles[0], previous, 2)
    testing.expect(t, p == current_p && f == current_f)
}

@(test)
test_capture_interpolation_is_deterministic_and_moves_between_ticks :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{100, 0}}}, []sim.Road{{0, 1, 7, 9}})
    defer sim.destroy(&world)
    append(&world.vehicles, sim.Vehicle{id = 1, road = 0, distance = 10, speed = 6, kind = .Car})
    previous := make(Render_Poses)
    defer delete(previous)
    last := Vec3{10, 0, 0}
    for frame in 1..=12 {
        advance_capture_frame(&world, frame, &previous)
        p, _ := interpolated_vehicle_pose(&world, world.vehicles[0], previous, f32(frame%3)/3)
        if frame > 3 { testing.expect(t, p[0] > last[0], "fixed-clock captures must use the same smooth presentation") }
        if frame == 4 || frame == 5 {
            raw, _ := vehicle_render_pose(&world, world.vehicles[0])
            testing.expect(t, p[0] > 10 && p[0] < raw[0])
        }
        last = p
    }
    testing.expect(t, math.abs(world.time-0.2) < 0.000001)
}
