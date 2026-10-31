package sim

import "core:testing"
import "core:fmt"

// Two three-abreast streams with intersecting paths. Reverse storage order as
// well: instantaneous-body-only admission used to let both noses get stuck.
@(test)
test_crossing_motorcycle_streams_use_local_gaps_and_drain :: proc(t: ^testing.T) {
    for reverse in 0..<2 {
        for initial_speed in ([]f32{0, 2, 5}) {
            w: World
            init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}},
                []Road{{0,1,3.5,8}, {1,2,3.5,8}, {3,1,3.5,8}, {1,4,3.5,8}}, 42)
            testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}, {2,4,86400,.Motorcycle}}))
            for approach in 0..<2 {
                for row in 0..<3 {
                    for band in 0..<3 {
                        lateral := source_lateral(3.5, .Motorcycle, band)
                        append(&w.vehicles, Vehicle{road = approach*2, spawn_index = approach,
                            distance = entry_distance(&w, approach*2, .Motorcycle)-f32(row)*3.5,
                            lateral = lateral, preferred_lateral = lateral, speed = initial_speed,
                            kind = .Motorcycle, id = u64(len(w.vehicles)+1)})
                    }
                }
            }
            if reverse == 1 {
                for i in 0..<len(w.vehicles)/2 {
                    j := len(w.vehicles)-1-i
                    w.vehicles[i], w.vehicles[j] = w.vehicles[j], w.vehicles[i]
                }
            }
            w.spawned, w.requested = 18, 18
            max_crossing := 0
            for _ in 0..<1600 {
                step(&w, 0.05)
                crossing := 0
                for v in w.vehicles { if v.motion.active && v.motion.node == 1 { crossing += 1 } }
                max_crossing = max(max_crossing, crossing)
                expect_bodies_clear(t, &w)
                testing.expect(t, w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
                if w.completed == 18 { break }
            }
            testing.expect(t, w.completed == 18 && len(w.motion_issues) == 0,
                fmt.tprintf("reverse=%d speed=%g completed=%d remaining=%v", reverse, initial_speed, w.completed, w.vehicles[:]))
            testing.expect(t, max_crossing > 3, "crossing streams must not require exclusive whole-node ownership")
            destroy(&w)
        }
    }
}

@(test)
test_turn_leader_can_enter_next_junction_while_follower_is_still_turning :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{0,18}}, {{60,18}}},
        []Road{{0,1,4,8}, {1,2,4,8}, {2,3,4,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,3,86400,.Motorcycle}}))
    entry := entry_distance(&w, 0, .Motorcycle)
    append(&w.vehicles,
        Vehicle{road = 0, distance = entry, speed = 2, kind = .Motorcycle, id = 1},
        Vehicle{road = 0, distance = entry-3.5, speed = 2, kind = .Motorcycle, id = 2})
    w.spawned, w.requested = 2, 2
    testing.expect(t, acquire_connector(&w, 0, 1))
    _, _ = move_vehicle(&w, 0, 4)
    _, _ = move_vehicle(&w, 1, 4)
    testing.expect(t, w.vehicles[1].motion.active)
    _, _ = move_vehicle(&w, 0, 20)
    testing.expect(t, w.vehicles[0].road == 1 && w.vehicles[0].motion.active && w.vehicles[0].motion.exit == 2,
        "a leader must not wait for a distant follower before turning at the next junction")
    testing.expect(t, w.vehicles[1].road == 0 && w.vehicles[1].motion.active)
    for _ in 0..<1600 { step(&w, 0.05); expect_bodies_clear(t, &w); if w.completed == 2 { break } }
    testing.expect(t, w.completed == 2 && len(w.motion_issues) == 0)
}

@(test)
test_receiving_source_uses_free_space_during_a_turn :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,-60}}, {{0,0}}, {{60,0}}}, []Road{{0,1,3.5,8}, {1,2,3.5,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,86400,.Motorcycle}, {1,2,0.05,.Motorcycle}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Motorcycle), kind = .Motorcycle, id = 1})
    w.spawned, w.requested = 1, 1
    testing.expect(t, acquire_connector(&w, 0, 1))
    scheduled_spawns(&w, 0.05)
    testing.expect(t, w.spawn_runtime[1].admitted == 1 && w.pending == 0,
        "a turning rider must not own the receiving strip ahead of its actual body")
    w.spawns[1].interval = 86400
    w.spawn_runtime[1].elapsed = 0
    for _ in 0..<1600 { step(&w, 0.05); expect_bodies_clear(t, &w); if w.completed == 2 { break } }
    testing.expect(t, w.completed == 2 && w.spawned == w.completed+len(w.vehicles) && w.requested == w.spawned+w.pending)
}
