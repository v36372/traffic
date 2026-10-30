package sim

import "core:math"
import "core:testing"
import "core:fmt"

expect_bodies_clear :: proc(t: ^testing.T, w: ^World) {
    for a, i in w.vehicles {
        for b in w.vehicles[i+1:] {
            testing.expect(t, !bodies_overlap(w, a, b, 0), "visible world-space body collision")
        }
    }
}

@(test)
test_angle_boundaries_red_queue_green_release :: proc(t: ^testing.T) {
    angles := []f32{0, math.PI*0.5, -math.PI*0.5, math.PI-0.01, -math.PI+0.01}
    for angle in angles {
        f := Vec2{math.cos(angle), math.sin(angle)}
        g := Vec2{math.cos(angle+0.4), math.sin(angle+0.4)}
        w: World
        init(&w, []Node{{{-60*f.x,-60*f.y}}, {{0,0}}, {{60*g.x,60*g.y}}}, []Road{
            {0,1,4,8}, {1,0,4,8}, {1,2,4,8}, {2,1,4,8},
        })
        testing.expect(t, configure(&w, []Signal_Plan{{1,2,1,1,2}}, []Spawn_Point{{0,2,86400,.Car}}))
        line := entry_distance(&w, 0, .Car)
        for i in 0..<3 { append(&w.vehicles, Vehicle{road = 0, distance = line-2-f32(i)*6, speed = 3, kind = .Car, id = u64(i+1)}) }
        w.spawned, w.requested = 3, 3
        // Starts amber, then stays red: no front may pass the shared bar.
        for _ in 0..<60 {
            step(&w, 0.05)
            for v in w.vehicles {
                testing.expect(t, v.road == 0 && !v.motion.active && v.distance <= line+0.001)
            }
            expect_bodies_clear(t, &w)
        }
        // Give this approach a long green; rotate through +/- pi without any
        // angle interpolation branch or renderer-side heading correction.
        w.signals[0].green, w.signals[0].offset, w.time = 100, 0, 0
        for _ in 0..<800 { step(&w, 0.05); expect_bodies_clear(t, &w) }
        testing.expect(t, w.completed == 3 && len(w.vehicles) == 0 && len(w.motion_issues) == 0)
        destroy(&w)
    }
}

@(test)
test_shared_node_phase_change_and_reserved_exit :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}}, []Road{
        {0,1,3.5,8}, {1,2,3.5,8}, {3,1,3.5,8}, {1,4,3.5,8},
    })
    defer destroy(&w)
    testing.expect(t, configure(&w, []Signal_Plan{{1,100,1,1,0}}, []Spawn_Point{{0,2,86400,.Car}, {2,4,86400,.Car}}))
    append(&w.vehicles, Vehicle{road = 0, distance = entry_distance(&w, 0, .Car), speed = 2, kind = .Car, id = 1})
    append(&w.vehicles, Vehicle{road = 2, distance = entry_distance(&w, 2, .Car), speed = 2, kind = .Car, spawn_index = 1, id = 2})
    w.spawned, w.requested = 2, 2
    step(&w, 0.05)
    testing.expect(t, w.vehicles[0].motion.active && !w.vehicles[1].motion.active)
    testing.expect(t, receiving_reserved(&w, 1, 0, .Motorcycle, 0))
    testing.expect(t, !free_lateral(&w, 1, 0, .Motorcycle, 0), "an outgoing source cannot invalidate a reserved exit")
    progress := w.vehicles[0].motion.progress
    w.signals[0].offset, w.time = 102, 0
    testing.expect(t, signal_state(&w, 0) == .Red && signal_state(&w, 2) == .Green)
    step(&w, 0.05)
    testing.expect(t, w.vehicles[0].motion.progress > progress && !w.vehicles[1].motion.active)
    saw_second, saw_release := false, false
    for _ in 0..<800 {
        step(&w, 0.05)
        owners := 0
        for v in w.vehicles {
            if v.motion.active { owners += 1 }
            if v.id == 1 && v.road == 1 {
                saw_release = true
                testing.expect(t, v.distance-kind_length(v.kind)*0.5 >= junction_mouth_depth(&w, 1)+0.999)
            }
            if v.id == 2 && v.motion.active { saw_second = true }
        }
        testing.expect(t, owners <= 1)
        expect_bodies_clear(t, &w)
    }
    testing.expect(t, saw_second && saw_release && w.completed == 2)
}

@(test)
test_blocked_exit_pending_and_recovery :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{100,0}}}, []Road{{0,1,2,8}, {1,2,2,0}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,0.5,.Car}}))
    append(&w.vehicles, Vehicle{road = 1, kind = .Car, id = 1})
    w.spawned, w.requested = 1, 1
    for _ in 0..<500 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        for v in w.vehicles { testing.expect(t, !v.motion.active) }
        testing.expect(t, w.requested == w.spawned+w.pending && w.spawned == w.completed+len(w.vehicles))
    }
    testing.expect(t, w.pending > 0 && w.completed == 0)
    // Stop new demand and release the actual blocking vehicle, not the queue.
    w.spawns[0].interval = 86400
    w.spawn_runtime[0].elapsed = 0
    w.roads[1].speed_limit = 8
    // One-body node serialization takes about five seconds per queued car;
    // this finite drain horizon includes the whole fifty-car pending backlog.
    for _ in 0..<8000 {
        step(&w, 0.05)
        expect_bodies_clear(t, &w)
        testing.expect(t, w.requested == w.spawned+w.pending && w.spawned == w.completed+len(w.vehicles))
    }
    testing.expect(t, w.pending == 0 && len(w.vehicles) == 0 && w.completed == w.requested,
        fmt.tprintf("recovery: pending=%d active=%d completed=%d requested=%d", w.pending, len(w.vehicles), w.completed, w.requested))
}

@(test)
test_unsupported_short_source_diagnosed_without_map_freeze :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{3,0}}, {{40,0}}, {{0,50}}, {{60,50}}}, []Road{{0,1,2,8}, {1,2,2,8}, {3,4,3,8}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{{0,2,0.5,.Car}, {2,4,2,.Car}}))
    for _ in 0..<600 { step(&w, 0.05) }
    testing.expect(t, w.spawn_runtime[0].admitted == 0 && w.spawn_runtime[0].pending > 0)
    testing.expect(t, w.completed > 0 && w.spawn_runtime[1].admitted > 0)
    testing.expect(t, len(w.motion_issues) == 1 && w.motion_issues[0] == Motion_Issue{0,1,.Short_Approach})
    testing.expect(t, w.requested == w.spawned+w.pending && w.spawned == w.completed+len(w.vehicles))

    // A manually present short-link vehicle stays at its real position. It
    // does not receive an endpoint-to-start teleport as an unsupported fallback.
    manual: World
    init(&manual, w.nodes[:3], w.roads[:2])
    defer destroy(&manual)
    manual.spawn_accumulator = -100
    append(&manual.vehicles, Vehicle{road = 0, kind = .Car, spawn_index = -1})
    before, _ := vehicle_pose(&manual, manual.vehicles[0])
    for _ in 0..<100 { step(&manual, 0.05) }
    after, _ := vehicle_pose(&manual, manual.vehicles[0])
    testing.expect(t, before == after && manual.vehicles[0].road == 0 && len(manual.motion_issues) == 1)
}

@(test)
test_motorcycle_source_bands_and_deterministic_reset :: proc(t: ^testing.T) {
    a, b: World
    nodes := []Node{{{0,0}}, {{70,0}}, {{140,0}}}
    roads := []Road{{0,1,3.5,8}, {1,2,3.5,8}}
    init(&a, nodes, roads, 42)
    init(&b, nodes, roads, 42)
    defer destroy(&a)
    defer destroy(&b)
    sources := []Spawn_Point{{0,2,1.2,.Motorcycle}}
    testing.expect(t, configure(&a, nil, sources) && configure(&b, nil, sources))
    for _ in 0..<400 { step(&a, 0.05); step(&b, 0.05) }
    bands: [3]int
    for v, i in a.vehicles {
        testing.expect(t, v == b.vehicles[i] && v.id > 0)
        testing.expect(t, v.preferred_lateral == source_lateral(3.5, .Motorcycle, int(v.id)-1))
        if v.preferred_lateral < -0.5 { bands[0] += 1 } else if v.preferred_lateral > 0.5 { bands[2] += 1 } else { bands[1] += 1 }
    }
    expect_bodies_clear(t, &a)
    testing.expect(t, bands[0] > 0 && bands[1] > 0 && bands[2] > 0)
    testing.expect(t, a.rng == 42 && b.rng == 42, "source bands must not consume physics RNG")
    // Destroy with reservations in flight, then reproduce the same IDs, poses,
    // counters and ownership from a clean init/configure lifecycle.
    destroy(&b)
    init(&b, nodes, roads, 42)
    testing.expect(t, configure(&b, nil, sources))
    testing.expect(t, b.spawned == 0 && len(b.motion_issues) == 0)
    for _ in 0..<400 { step(&b, 0.05) }
    testing.expect(t, a.spawned == b.spawned && a.completed == b.completed && a.pending == b.pending && len(a.vehicles) == len(b.vehicles))
    for v, i in a.vehicles { testing.expect(t, v == b.vehicles[i]) }
}
