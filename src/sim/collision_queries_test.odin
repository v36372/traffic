package sim

import "core:testing"

// Uncached 30-sample oracle: optimization must not change any admission decision,
// including follower/entry priority, stopped traffic, lane changes or connector exits.
uncached_gap_clear :: proc(w: ^World, candidate: Vehicle, ignore: int, speed: f32) -> bool {
    for other, j in w.vehicles {
        if j == ignore { continue }
        if other.road == candidate.road && !other.motion.active && !other.lane.active &&
            other.distance < candidate.distance && abs(other.lateral-candidate.lateral) < (kind_width(other.kind)+kind_width(candidate.kind))*0.5+0.18 { continue }
        if other.motion.active && candidate.motion.sequence != 0 && other.motion.sequence > candidate.motion.sequence { continue }
        p, _ := vehicle_pose(w, candidate)
        q, _ := vehicle_pose(w, other)
        reach := (speed+other.speed)*1.5+kind_length(candidate.kind)+kind_length(other.kind)
        if vec_distance(p, q) > reach { continue }
        for k in 1..=30 {
            time := f32(k)*0.05
            a := forecast_vehicle(w, candidate, speed*time)
            b := forecast_vehicle(w, other, other.speed*time)
            if bodies_overlap(w, a, b) { return false }
        }
    }
    return true
}

@(test)
test_cached_collision_queries_match_uncached_predictions :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-60,0}}, {{0,0}}, {{60,0}}, {{0,-60}}, {{0,60}}},
        []Road{{0,1,7,9}, {1,0,7,9}, {1,2,7,9}, {2,1,7,9}, {3,1,7,9}, {1,3,7,9}, {1,4,7,9}, {4,1,7,9}})
    defer destroy(&w)
    testing.expect(t, configure(&w, nil, []Spawn_Point{
        {0,4,86400,.Motorcycle}, {3,0,86400,.Car}, {4,2,86400,.Bicycle}, {7,3,86400,.Bus},
    }))
    for source, i in w.spawns {
        for row in 0..<3 {
            append(&w.vehicles, Vehicle{road = source.road, spawn_index = i, id = u64(len(w.vehicles)+1),
                distance = entry_distance(&w, source.road, source.kind)-f32(row)*12,
                kind = source.kind, speed = f32(row)*2})
        }
    }
    w.spawned, w.requested = len(w.vehicles), len(w.vehicles)
    saw_connector, saw_clear, saw_blocked, saw_lane := false, false, false, false
    for tick in 0..<400 {
        if tick%10 == 0 {
            for original, i in w.vehicles {
                for lane in 0..<2 {
                    candidate := original
                    if lane == 1 && !candidate.motion.active {
                        candidate.lane = {active = true, start = candidate.distance-3, from = 0, to = 1}
                        candidate.lateral, _ = lane_offset(candidate)
                        saw_lane = true
                    }
                    for speed in ([]f32{0, 2, 5, 9}) {
                        // Both ignored-self and new-admission queries; the latter
                        // includes an actual overlapping body in the world.
                        for ignore in ([2]int{i, -1}) {
                            actual := local_gap_clear(&w, candidate, ignore, speed)
                            testing.expect(t, actual == uncached_gap_clear(&w, candidate, ignore, speed))
                            saw_clear = saw_clear || actual
                            saw_blocked = saw_blocked || !actual
                            expected_clear := true
                            for other, j in w.vehicles {
                                if j != ignore && bodies_overlap(&w, candidate, other) { expected_clear = false; break }
                            }
                            testing.expect(t, body_clear(&w, candidate, ignore) == expected_clear)
                        }
                    }
                    saw_connector = saw_connector || candidate.motion.active
                }
            }
        }
        step(&w, 0.05)
    }
    testing.expect(t, saw_connector && saw_lane && saw_clear && saw_blocked)
}
