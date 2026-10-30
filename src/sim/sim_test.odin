package sim

import "core:testing"

@(test)
test_demo_deterministic_and_shared_width :: proc(t: ^testing.T) {
    a, b: World
    init_demo(&a, 42)
    init_demo(&b, 42)
    defer destroy(&a)
    defer destroy(&b)
    for _ in 0..<800 {
        step(&a, 0.05)
        step(&b, 0.05)
    }
    testing.expect(t, len(a.vehicles) > 0)
    testing.expect(t, a.completed > 0)
    testing.expect(t, a.spawned == len(a.vehicles) + a.completed)
    testing.expect(t, a.spawned == b.spawned && a.completed == b.completed)
    testing.expect(t, len(a.vehicles) == len(b.vehicles))
    for i in 0..<len(a.vehicles) {
        testing.expect(t, a.vehicles[i] == b.vehicles[i])
        r := a.roads[a.vehicles[i].road]
        testing.expect(t, abs(a.vehicles[i].lateral) + kind_width(a.vehicles[i].kind)*0.5 <= r.width*0.5 + 0.001)
        testing.expect(t, a.vehicles[i].speed >= 0)
    }
}

@(test)
test_safe_following_and_queue :: proc(t: ^testing.T) {
    nodes := []Node{{{0, 0}}, {{40, 0}}, {{80, 0}}}
    roads := []Road{{0, 1, 3.0, 6.0}, {1, 2, 3.0, 6.0}}
    w: World
    init(&w, nodes, roads, 7)
    defer destroy(&w)
    append(&w.vehicles, Vehicle{road = 0, distance = 4.6, kind = .Car, spawn_index = -1})
    append(&w.vehicles, Vehicle{road = 0, kind = .Car, spawn_index = -1})
    for _ in 0..<100 {
        step(&w, 0.05)
        for i in 0..<len(w.vehicles) {
            for j in i+1..<len(w.vehicles) {
                x, y := w.vehicles[i], w.vehicles[j]
                if x.road == y.road && abs(x.lateral-y.lateral) < (kind_width(x.kind)+kind_width(y.kind))*0.5 + 0.18 {
                    testing.expect(t, abs(x.distance-y.distance) >= (kind_length(x.kind)+kind_length(y.kind))*0.5 - 0.01)
                }
            }
        }
    }
}

@(test)
test_narrowing_transition_and_blocked_recovery :: proc(t: ^testing.T) {
    // Long enough for a 4.5m body, the connector and a receiving reservation.
    nodes := []Node{{{0, 0}}, {{40, 0}}, {{120, 0}}}
    roads := []Road{{0, 1, 6.0, 8.0}, {1, 2, 2.0, 8.0}}
    narrow: World
    init(&narrow, nodes, roads, 9)
    defer destroy(&narrow)
    narrow.spawn_accumulator = -100
    append(&narrow.vehicles, Vehicle{road = 0, distance = 29, lateral = 1.5, preferred_lateral = 1.5, speed = 4, kind = .Car, spawn_index = -1})
    saw_tail := false
    for _ in 0..<200 {
        step(&narrow, 0.05)
        c := narrow.vehicles[0].motion
        if c.active && c.tail > 0 && c.progress > c.lead+c.arc[64] {
            saw_tail = true
            _, f := vehicle_pose(&narrow, narrow.vehicles[0])
            testing.expect(t, f.x > 0.9999 && abs(f.y) < 0.0001 && receiving_reserved(&narrow, 1, 0, .Car, 0))
            a, af := connector_pose(c, c.lead+c.arc[64]-0.001)
            b, bf := connector_pose(c, c.lead+c.arc[64]+0.001)
            testing.expect(t, vec_distance(a, b) < 0.003 && af.x*bf.x+af.y*bf.y > 0.999)
        }
        if narrow.vehicles[0].road == 1 { break }
    }
    testing.expect(t, narrow.vehicles[0].road == 1)
    testing.expect(t, saw_tail, "finish steering before entering the narrow exit, but retain reservation through the straight tail")
    testing.expect(t, abs(narrow.vehicles[0].lateral) <= 0.05+0.001)

    blocked: World
    init(&blocked, []Node{{{0, 0}}, {{40, 0}}, {{120, 0}}}, []Road{{0, 1, 2.0, 6.0}, {1, 2, 2.0, 0.0}}, 10)
    defer destroy(&blocked)
    blocked.spawn_accumulator = -100
    append(&blocked.vehicles, Vehicle{road = 0, distance = entry_distance(&blocked, 0, .Car)-1, speed = 3, kind = .Car, spawn_index = -1})
    append(&blocked.vehicles, Vehicle{road = 1, kind = .Car, spawn_index = -1})
    for _ in 0..<50 { step(&blocked, 0.05) }
    testing.expect(t, blocked.vehicles[0].road == 0)
    testing.expect(t, !blocked.vehicles[0].motion.active && blocked.vehicles[0].distance+kind_length(.Car)*0.5 <= stop_line_distance(&blocked, 0)+0.001)
    blocked.vehicles[1].distance = 20
    blocked.roads[1].speed_limit = 6
    for _ in 0..<200 {
        step(&blocked, 0.05)
        testing.expect(t, !bodies_overlap(&blocked, blocked.vehicles[0], blocked.vehicles[1]))
        if blocked.vehicles[0].road == 1 { break }
    }
    testing.expect(t, blocked.vehicles[0].road == 1)
}

@(test)
test_prefers_non_uturn :: proc(t: ^testing.T) {
    nodes := []Node{{{0, 0}}, {{1, 0}}, {{2, 0}}, {{2, 1}}}
    roads := []Road{{0, 1, 4, 5}, {1, 0, 4, 5}, {1, 2, 4, 5}, {2, 1, 4, 5}, {2, 3, 4, 5}}
    w: World
    init(&w, nodes, roads)
    defer destroy(&w)
    testing.expect(t, next_road(&w, 0) == 2)
    testing.expect(t, next_road(&w, 2) == 4)
}

@(test)
test_seeded_invariants_every_tick :: proc(t: ^testing.T) {
    for seed in u64(1)..=u64(8) {
        a, b: World
        init_demo(&a, seed)
        init_demo(&b, seed)
        for _ in 0..<3000 {
            step(&a, 0.05)
            step(&b, 0.05)
            testing.expect(t, a.spawned == a.completed + len(a.vehicles))
            testing.expect(t, a.spawned == b.spawned && a.completed == b.completed && len(a.vehicles) == len(b.vehicles))
            for i in 0..<len(a.vehicles) {
                v := a.vehicles[i]
                testing.expect(t, v == b.vehicles[i])
                testing.expect(t, v.road >= 0 && v.road < len(a.roads))
                testing.expect(t, v.distance >= 0 && v.distance <= road_length(&a, v.road) + 0.001)
                testing.expect(t, abs(v.lateral) + kind_width(v.kind)*0.5 <= a.roads[v.road].width*0.5 + 0.001)
                for j in i+1..<len(a.vehicles) {
                    other := a.vehicles[j]
                    if v.road == other.road && abs(v.lateral-other.lateral) < (kind_width(v.kind)+kind_width(other.kind))*0.5 + 0.18 {
                        testing.expect(t, abs(v.distance-other.distance) >= (kind_length(v.kind)+kind_length(other.kind))*0.5 - 0.001)
                    }
                }
            }
        }
        testing.expect(t, a.completed > 0)
        destroy(&a)
        destroy(&b)
    }
}

@(test)
test_synthetic_spawn_mix :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0, 0}}, {{100, 0}}}, []Road{{0, 1, 10, 10}}, 42)
    defer destroy(&w)
    motorcycles := 0
    for _ in 0..<10000 {
        try_spawn(&w)
        testing.expect(t, len(w.vehicles) == 1)
        if w.vehicles[0].kind == .Motorcycle { motorcycles += 1 }
        clear(&w.vehicles)
    }
    testing.expect(t, motorcycles > 6200 && motorcycles < 6800)
}
