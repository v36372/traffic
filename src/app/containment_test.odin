package app

import "core:fmt"
import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_authored_turn_bodies_stay_on_actual_asphalt_union :: proc(t: ^testing.T) {
    for widths in ([][4]f32{{3.5, 3.5, 3.5, 3.5}, {5.5, 5.5, 3.5, 3.5}, {3, 6, 9, 4}}) {
        world: sim.World
        sim.init(&world, []sim.Node{{{0, 0}}, {{-120, 0}}, {{120, 0}}, {{0, 120}}, {{0, -120}}},
            []sim.Road{{1, 0, widths[0], 9}, {0, 1, widths[1], 9}, {0, 2, widths[1], 9}, {2, 0, widths[0], 9},
                       {3, 0, widths[2], 9}, {0, 3, widths[3], 9}, {0, 4, widths[3], 9}, {4, 0, widths[2], 9}})
        defer sim.destroy(&world)
        offsets := road_offsets(&world)
        defer delete(offsets)
        scene := build_road_scene(&world, offsets)
        defer destroy_road_scene(&scene)
        for kind in ([]sim.Vehicle_Kind{.Motorcycle, .Car, .Bus}) {
            for exit in ([]int{5, 6}) { // west incoming -> north left / south right
                vehicle := sim.Vehicle{road = 0, kind = kind, lateral = 0}
                connector, problem := sim.make_connector(&world, vehicle, exit, 0)
                if !testing.expect(t, problem == .None, fmt.tprintf("connector unsupported: widths=%v kind=%v exit=%d reason=%v", widths, kind, exit, problem)) { continue }
                vehicle.motion = connector
                samples := int(math.ceil(connector.length/0.1))
                contained := true
                for step in 0..=samples {
                    vehicle.motion.progress = connector.length*f32(step)/f32(samples)
                    p, f := sim.vehicle_pose(&world, vehicle)
                    for along in ([]f32{-0.5, 0, 0.5}) {
                        for across in ([]f32{-0.5, 0, 0.5}) {
                            if along == 0 && across == 0 { continue }
                            longitudinal, lateral := along*sim.kind_length(kind), across*sim.kind_width(kind)
                            // All four body corners + all four edge centers.
                            point := Vec3{p.x+f.x*longitudinal-f.y*lateral, 0, -(p.y+f.y*longitudinal+f.x*lateral)}
                            if !testing.expect(t, road_point_covered(&scene, 2, point), fmt.tprintf("OFF ASPHALT widths=%v kind=%v exit=%d progress=%.4f/%.4f corner=(%.1f,%.1f) point=(%.5f,%.5f)", widths, kind, exit, vehicle.motion.progress, connector.length, along, across, point[0], -point[2])) {
                                contained = false
                                break
                            }
                        }
                        if !contained { break }
                    }
                    if !contained { break }
                }
            }
        }
    }
}

@(test)
test_terminal_bus_rejects_front_spill_but_authored_forward_strip_fits :: proc(t: ^testing.T) {
    for forward_strip in ([]bool{false, true}) {
        nodes := []sim.Node{{{-60, 0}}, {{0, 0}}, {{0, -60}}, {{60, 0}}}
        input := []sim.Road{{0, 1, 6, 9}, {2, 1, 6, 9}, {1, 3, 6, 9}}
        count := 2
        if forward_strip { count = 3 }
        world: sim.World
        sim.init(&world, nodes, input[:count])
        defer sim.destroy(&world)
        testing.expect(t, sim.configure(&world, []sim.Signal_Plan{{1, 100, 1, 1, 0}}, []sim.Spawn_Point{{0, 1, 1, .Bus}}))
        offsets := road_offsets(&world)
        defer delete(offsets)
        scene := build_road_scene(&world, offsets)
        defer destroy_road_scene(&scene)
        bus := sim.Vehicle{road = 0, kind = .Bus, spawn_index = 0}
        connector, problem := sim.make_connector(&world, bus, -1, 0)
        if !forward_strip {
            // Rear clearance alone would accept: the rear is at x=-5 while
            // the band ends at x=-7. But the front x=5 exceeds asphalt x=3.
            _, far, _, _, ok := sim.crossing_band(&world, 0)
            testing.expect(t, ok && 60-sim.kind_length(.Bus)*0.5 > far+0.1)
            testing.expect(t, !road_point_covered(&scene, 2, {5, 0, 0}), "terminal bus front would leave actual perpendicular asphalt")
            testing.expect(t, problem == .Terminal_Clearance, "rear-clear but off-pavement terminal bus must be explicitly rejected")
            for _ in 0..<40 { sim.step(&world, f32(TICK)) }
            testing.expect(t, world.pending > 0 && world.spawned == 0 && world.completed == 0)
            testing.expect(t, len(world.motion_issues) == 1 && world.motion_issues[0].problem == .Terminal_Clearance)
            continue
        }
        if !testing.expect(t, problem == .None, "authored forward strip makes the full terminal bus path feasible") { continue }
        bus.motion = connector
        samples := int(math.ceil(connector.length/0.1))
        for step in 0..=samples {
            bus.motion.progress = connector.length*f32(step)/f32(samples)
            p, f := sim.vehicle_pose(&world, bus)
            for along in ([]f32{-0.5, 0, 0.5}) {
                for across in ([]f32{-0.5, 0, 0.5}) {
                    if along == 0 && across == 0 { continue }
                    x, y := along*sim.kind_length(.Bus), across*sim.kind_width(.Bus)
                    point := Vec3{p.x+f.x*x-f.y*y, 0, -(p.y+f.y*x+f.x*y)}
                    testing.expect(t, road_point_covered(&scene, 2, point), "all eight terminal bus body samples must stay on actual cached asphalt for the entire owned connector")
                }
            }
        }
    }
}
