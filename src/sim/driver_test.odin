package sim

import "core:testing"

@(test)
test_driver_samples_and_configuration :: proc(t: ^testing.T) {
    a, b: World
    init_demo(&a, 42)
    init_demo(&b, 42)
    defer destroy(&a)
    defer destroy(&b)
    config := default_driver_config()
    config.enabled = true
    config.classes[int(Vehicle_Kind.Motorcycle)] = {0.5, 1, 0.4, 2}
    testing.expect(t, configure_drivers(&a, config) && configure_drivers(&b, config))
    invalid := config
    invalid.classes[0].speed_max = 1.1
    testing.expect(t, !configure_drivers(&a, invalid) && a.behavior == config)
    first, ok := make_driver(config, a.behavior_seed, 1, .Motorcycle, .Balanced)
    same, _ := make_driver(config, b.behavior_seed, 1, .Motorcycle, .Balanced)
    other, _ := make_driver(config, a.behavior_seed, 2, .Motorcycle, .Balanced)
    car, _ := make_driver(config, a.behavior_seed, 1, .Car, .Balanced)
    testing.expect(t, ok && first == same && first.traits != other.traits)
    testing.expect(t, first.traits.speed_factor >= 0.5 && first.traits.speed_factor <= 1 && first.traits.headway >= 0.4 && first.traits.headway <= 2)
    testing.expect(t, first.traits.acceleration >= 1.4 && first.traits.acceleration <= 2.6)
    testing.expect(t, first.traits.acceleration != other.traits.acceleration)
    testing.expect(t, first.aggressiveness >= 0.25 && first.aggressiveness <= 0.7)
    testing.expect(t, first.patience > 0 && first.wander > 0)
    testing.expect(t, car.traits.speed_factor == 1 && car.traits.headway == 0.8 && car.traits.acceleration == 2)
    testing.expect(t, car.aggressiveness >= 0.25 && car.aggressiveness <= 0.7)
    original_rng := a.rng
    _, _ = make_driver(config, a.behavior_seed, 1, .Motorcycle, .Balanced)
    testing.expect(t, a.rng == original_rng)
    b.behavior_seed += 1
    changed, _ := make_driver(config, b.behavior_seed, 1, .Motorcycle, .Balanced)
    testing.expect(t, first.traits != changed.traits)
}

@(test)
test_varied_drivers_replay_persist_and_clearance :: proc(t: ^testing.T) {
    a, b: World
    init_demo(&a, 77)
    init_demo(&b, 77)
    defer destroy(&a)
    defer destroy(&b)
    config := default_driver_config()
    config.enabled = true
    for &r in config.classes { r = {0.5, 1, 0.4, 2} }
    testing.expect(t, configure_drivers(&a, config) && configure_drivers(&b, config))
    for _ in 0..<600 {
        step(&a, 0.05)
        step(&b, 0.05)
        testing.expect(t, a.spawned == b.spawned && len(a.vehicles) == len(b.vehicles))
        for v, i in a.vehicles {
            initial, ok := make_driver(a.behavior, a.behavior_seed, v.id, v.kind, spawn_persona(a.behavior, a.behavior_seed, v.id))
            testing.expect(t, ok && v == b.vehicles[i] && v.driver.traits == initial.traits && v.driver.persona == initial.persona && v.driver.kind == initial.kind && v.driver.side == initial.side && v.driver.aggressiveness == initial.aggressiveness && v.driver.wander == initial.wander)
            for other in a.vehicles[i+1:] {
                testing.expect(t, !bodies_overlap(&a, v, other))
            }
        }
    }
    testing.expect(t, a.spawned > 0 && !configure_drivers(&a, config))
}

@(test)
test_default_driver_configuration_matches_baseline :: proc(t: ^testing.T) {
    a, b: World
    init_demo(&a, 7)
    init_demo(&b, 7)
    defer destroy(&a)
    defer destroy(&b)
    config := default_driver_config()
    config.enabled = true
    testing.expect(t, configure_drivers(&b, config))
    for _ in 0..<300 {
        step(&a, 0.05)
        step(&b, 0.05)
        testing.expect(t, a.rng == b.rng && a.spawned == b.spawned && a.completed == b.completed && len(a.vehicles) == len(b.vehicles))
        for v, i in a.vehicles {
            other := b.vehicles[i]
            other.driver = v.driver
            testing.expect(t, v == other)
        }
    }
}

@(test)
test_traits_affect_speed_and_following :: proc(t: ^testing.T) {
    nodes := []Node{{{0, 0}}, {{200, 0}}}
    roads := []Road{{0, 1, 3, 7}}
    fast, slow, close, distant: World
    init(&fast, nodes, roads); init(&slow, nodes, roads)
    init(&close, nodes, roads); init(&distant, nodes, roads)
    defer destroy(&fast); defer destroy(&slow); defer destroy(&close); defer destroy(&distant)
    config := default_driver_config()
    config.enabled = true
    testing.expect(t, configure_drivers(&fast, config) && configure_drivers(&slow, config) && configure_drivers(&close, config) && configure_drivers(&distant, config))
    for w in ([]^World{&fast, &slow, &close, &distant}) { w.spawn_accumulator = -100 }
    append(&fast.vehicles, Vehicle{road = 0, distance = 20, speed = 5, kind = .Car, spawn_index = -1, driver = {traits = {1, 0.8, 2}}})
    append(&slow.vehicles, Vehicle{road = 0, distance = 20, speed = 5, kind = .Car, spawn_index = -1, driver = {traits = {0.5, 0.8, 2}}})
    for w in ([]^World{&close, &distant}) {
        append(&w.vehicles, Vehicle{road = 0, distance = 27.5, kind = .Car, spawn_index = -1, driver = {traits = {1, 0.8, 2}}})
        append(&w.vehicles, Vehicle{road = 0, distance = 20, speed = 4, kind = .Car, spawn_index = -1, driver = {traits = {1, 0.4, 2}}})
    }
    distant.vehicles[1].driver.traits.headway = 2
    // Both close followers initially need maximum braking; compare their
    // preferred gaps after reacting, not during that shared emergency tick.
    for w in ([]^World{&fast, &slow, &close, &distant}) { for _ in 0..<20 { step(w, 0.05) } }
    testing.expect(t, fast.vehicles[0].speed > slow.vehicles[0].speed)
    testing.expect(t, close.vehicles[1].speed > distant.vehicles[1].speed)
}

@(test)
test_individual_acceleration_changes_motion :: proc(t: ^testing.T) {
    nodes := []Node{{{0, 0}}, {{200, 0}}}
    roads := []Road{{0, 1, 3, 8}}
    slow, fast: World
    init(&slow, nodes, roads); init(&fast, nodes, roads)
    defer destroy(&slow); defer destroy(&fast)
    config := default_driver_config(); config.enabled = true
    testing.expect(t, configure_drivers(&slow, config) && configure_drivers(&fast, config))
    slow.spawn_accumulator, fast.spawn_accumulator = -100, -100
    append(&slow.vehicles, Vehicle{road = 0, distance = 20, kind = .Motorcycle, spawn_index = -1, driver = {traits = {1, 0.8, 1.4}}})
    append(&fast.vehicles, Vehicle{road = 0, distance = 20, kind = .Motorcycle, spawn_index = -1, driver = {traits = {1, 0.8, 2.6}}})
    for _ in 0..<20 { step(&slow, 0.05); step(&fast, 0.05) }
    testing.expect(t, fast.vehicles[0].speed > slow.vehicles[0].speed+1)
    testing.expect(t, abs(slow.vehicles[0].speed-1.4) < 0.01 && abs(fast.vehicles[0].speed-2.6) < 0.01)
}

@(test)
test_driver_factory_validation_and_independence :: proc(t: ^testing.T) {
    config := default_driver_config(); config.enabled = true
    config.classes[int(Vehicle_Kind.Car)] = {0.6, 1, 0.4, 2}
    a, ok := make_driver(config, 17, 1, .Car, .Cautious)
    b, ok2 := make_driver(config, 17, 1, .Car, .Cautious)
    testing.expect(t, ok && ok2 && a == b && a.mode == .Cruising)
    a.mode = .Passing
    testing.expect(t, b.mode == .Cruising && a.traits == b.traits)
    c, _ := make_driver(config, 17, 2, .Car, .Cautious)
    testing.expect(t, c.traits != b.traits)
    _, ok = make_driver(config, 17, 0, .Car, .Balanced); testing.expect(t, !ok)
    _, ok = make_driver(config, 17, 1, Vehicle_Kind(99), .Balanced); testing.expect(t, !ok)
    _, ok = make_driver(config, 17, 1, .Car, Driver_Persona(99)); testing.expect(t, !ok)
    config.classes[0].speed_max = 2
    _, ok = make_driver(config, 17, 1, .Car, .Balanced); testing.expect(t, !ok)
    baseline, base_ok := make_driver(Driver_Config{}, 17, 1, .Car, .Balanced)
    testing.expect(t, base_ok && baseline.traits == Driver_Traits{1, 0.8, 2})
    testing.expect(t, baseline.aggressiveness == 0 && baseline.wander == 0)
}

@(test)
test_persona_and_kind_change_passing_choice :: proc(t: ^testing.T) {
    for persona in ([]Driver_Persona{.Assertive, .Balanced, .Cautious}) {
        w: World; passing_world(&w, false); defer destroy(&w)
        w.roads[0].width = 20
        w.vehicles[0].speed = 7.5
        d, ok := make_driver(w.behavior, w.behavior_seed, 1, .Motorcycle, persona)
        d.traits.speed_factor = 1
        testing.expect(t, ok)
        w.vehicles[1].driver = d
        cache_geometry(&w); try_overtake(&w, 1)
        testing.expect(t, w.vehicles[1].lane.active == (persona != .Cautious))
    }
    for kind in ([]Vehicle_Kind{.Car, .Truck}) {
        w: World; passing_world(&w, false); defer destroy(&w)
        w.roads[0].width = 20
        w.vehicles[0].speed = kind_speed(kind)-1.25
        w.vehicles[1].kind = kind
        d, ok := make_driver(w.behavior, w.behavior_seed, 1, kind, .Balanced)
        testing.expect(t, ok)
        w.vehicles[1].driver = d
        cache_geometry(&w); try_overtake(&w, 1)
        testing.expect(t, w.vehicles[1].lane.active == (kind == .Car))
    }
}

@(test)
test_explicit_persona_works_with_fixed_ranges :: proc(t: ^testing.T) {
    for persona in ([]Driver_Persona{.Assertive, .Cautious}) {
        w: World
        passing_world(&w, false)
        defer destroy(&w)
        lead, follower := w.vehicles[0], w.vehicles[1]
        clear(&w.vehicles)
        config := default_driver_config(); config.enabled = true
        testing.expect(t, configure_drivers(&w, config))
        append(&w.vehicles, lead, follower)
        w.vehicles[0].speed = 7.5
        driver, ok := make_driver(config, w.behavior_seed, 1, .Motorcycle, persona)
        testing.expect(t, ok)
        w.vehicles[1].driver = driver
        cache_geometry(&w)
        try_overtake(&w, 1)
        testing.expect(t, w.vehicles[1].lane.active == (persona == .Assertive))
    }
}

@(test)
test_driver_modes_track_maneuvers :: proc(t: ^testing.T) {
    w: World; passing_world(&w, false); defer destroy(&w)
    cache_geometry(&w)
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Following)
    try_overtake(&w, 1); testing.expect(t, w.vehicles[1].lane.active)
    w.vehicles[1].lane = {}; w.vehicles[0].distance = 100
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Cruising)
    w.vehicles[0].distance = 55
    w.vehicles[1].driver.patience, w.vehicles[1].driver.waited = 1, 1
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Seeking)
    w.vehicles[1].driver.waited = 0
    w.vehicles[1].lane = {active = true, start = 30, from = 0, to = 1, length = 6}
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Filtering)
    w.vehicles[1].lane = {}
    w.vehicles[1].motion.active = true
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Junction)
    w.vehicles[1].motion = {}
    append(&w.signals, Signal_Plan{node = 1, green = 1, amber = 1, clearance = 100, offset = 2})
    cache_geometry(&w)
    w.vehicles[1].distance = entry_distance(&w, 0, .Motorcycle)
    w.vehicles[1].speed = 0
    update_driver_mode(&w, 1); testing.expect(t, w.vehicles[1].driver.mode == .Stopped_Signal)
}

@(test)
test_factory_used_by_both_spawn_paths :: proc(t: ^testing.T) {
    for scheduled in ([]bool{false, true}) {
        w: World
        init(&w, []Node{{{0, 0}}, {{300, 0}}}, []Road{{0, 1, 8, 9}}, 22)
        defer destroy(&w)
        config := default_driver_config(); config.enabled = true
        config.classes[int(Vehicle_Kind.Car)] = {0.7, 1, 0.5, 1.5}
        testing.expect(t, configure_drivers(&w, config))
        cache_geometry(&w)
        if scheduled {
            testing.expect(t, configure(&w, nil, []Spawn_Point{{road = 0, destination = 1, interval = 1, kind = .Car}}))
            scheduled_spawns(&w, 1)
        } else {
            for _ in 0..<100 { if len(w.vehicles) > 0 { break }; try_spawn(&w) }
        }
        testing.expect(t, len(w.vehicles) > 0)
        if len(w.vehicles) > 0 {
            v := w.vehicles[0]
            expected, ok := make_driver(config, 22, v.id, v.kind, spawn_persona(config, 22, v.id))
            testing.expect(t, ok && v.driver == expected)
        }
    }
}

@(test)
test_spawn_personas_are_mixed_and_stable :: proc(t: ^testing.T) {
    config := default_driver_config(); config.enabled = true
    config.classes[0].speed_min = 0.5
    seen: [3]bool
    for id in u64(1)..=u64(100) {
        persona := spawn_persona(config, 22, id)
        seen[int(persona)] = true
        testing.expect(t, persona == spawn_persona(config, 22, id))
    }
    testing.expect(t, seen[0] && seen[1] && seen[2])
    config = default_driver_config(); config.enabled = true
    for id in u64(1)..=u64(20) { testing.expect(t, spawn_persona(config, 22, id) == .Balanced) }
}

@(test)
test_aggressiveness_bands_and_patience_change_passing :: proc(t: ^testing.T) {
    config := default_driver_config(); config.enabled = true
    config.classes[int(Vehicle_Kind.Motorcycle)] = {0.65, 1, 0.5, 1.5}
    cautious, ok := make_driver(config, 9, 1, .Motorcycle, .Cautious)
    assertive, ok2 := make_driver(config, 9, 1, .Motorcycle, .Assertive)
    testing.expect(t, ok && ok2)
    testing.expect(t, cautious.aggressiveness >= 0.05 && cautious.aggressiveness <= 0.35)
    testing.expect(t, assertive.aggressiveness >= 0.6 && assertive.aggressiveness <= 1)
    testing.expect(t, assertive.patience < cautious.patience)
    testing.expect(t, passing_advantage(assertive) < passing_advantage(cautious))
    waiting := cautious
    waiting.waited = waiting.patience
    testing.expect(t, passing_advantage(waiting) <= 0.12)
    testing.expect(t, pass_length(Vehicle{kind = .Motorcycle, driver = assertive}) < LANE_CHANGE_LENGTH)
    testing.expect(t, pass_length(Vehicle{kind = .Motorcycle, driver = {}}) == LANE_CHANGE_LENGTH)
    testing.expect(t, gap_horizon(assertive) < gap_horizon(cautious) && gap_horizon(Driver{}) == 1.5)
}

@(test)
test_synthetic_successors_vary_with_wander :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{40,0}}, {{80,0}}, {{40,40}}}, []Road{{0,1,4,8}, {1,2,4,8}, {1,3,4,8}})
    defer destroy(&w)
    testing.expect(t, next_road(&w, 0) == 1)
    primary := Vehicle{road = 0, id = 1, kind = .Motorcycle}
    testing.expect(t, choose_next_road(&w, primary) == 1)
    seen_turn, seen_straight := false, false
    for id in u64(1)..=u64(80) {
        v := Vehicle{road = 0, id = id, kind = .Motorcycle, driver = {wander = 1}}
        exit := choose_next_road(&w, v)
        if exit == 1 { seen_straight = true } else if exit == 2 { seen_turn = true }
        testing.expect(t, exit == choose_next_road(&w, v))
        testing.expect(t, w.rng == 1)
    }
    testing.expect(t, seen_straight && seen_turn)
}

@(test)
test_turn_bias_prefers_inside_receiving_band :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{-40,0}}, {{0,0}}, {{0,40}}}, []Road{{0,1,6,8}, {1,2,6,8}})
    defer destroy(&w)
    cache_geometry(&w)
    left := Vehicle{road = 0, preferred_lateral = 0, kind = .Motorcycle, driver = {turn_bias = 1}}
    right := left
    right.driver.turn_bias = -1
    inside := connector_laterals(&w, left, 1)
    outside := connector_laterals(&w, right, 1)
    defer delete(inside)
    defer delete(outside)
    testing.expect(t, len(inside) > 0 && inside[0] > 0.5)
    testing.expect(t, len(outside) > 0 && outside[0] < -0.5)
}

@(test)
test_clear_parallel_course_does_not_stop_for_adjacent_body :: proc(t: ^testing.T) {
    w: World
    init(&w, []Node{{{0,0}}, {{300,0}}}, []Road{{0,1,8,9}})
    defer destroy(&w)
    w.spawn_accumulator = -1000
    config := default_driver_config()
    config.enabled = true
    testing.expect(t, configure_drivers(&w, config))
    driver, _ := make_driver(config, 1, 2, .Car, .Balanced)
    append(&w.vehicles,
        Vehicle{road = 0, distance = 41, kind = .Car, id = 1},
        Vehicle{road = 0, distance = 40, lateral = 2, speed = 2, kind = .Car, id = 2, driver = driver})
    step(&w, 0.05)
    testing.expect(t, w.vehicles[1].distance > 40 && !w.vehicles[1].lane.active,
        "a clear parallel body corridor needs neither braking nor a new lateral maneuver")
    testing.expect(t, !bodies_overlap(&w, w.vehicles[0], w.vehicles[1]))
}
