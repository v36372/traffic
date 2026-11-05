package sim
import "core:math"

Vec2 :: struct { x, y: f32 }
Node :: struct { position: Vec2 }
Road :: struct { from, to: int, width: f32, speed_limit: f32 }
Vehicle_Kind :: enum { Motorcycle, Car, Bus, Bicycle, Truck, Tricycle }
Vehicle :: struct {
    road: int,
    distance, lateral, speed: f32,
    kind: Vehicle_Kind,
    spawn_index: int,
    id: u64,
    preferred_lateral: f32,
    driver: Driver,
    motion: Connector,
    lane: Lane_Change,
}
Signal_Plan :: struct { node: int, green, amber, clearance, offset: f32, countdown: bool }
Spawn_Point :: struct { road, destination: int, interval: f32, kind: Vehicle_Kind }
Signal_State :: enum { Uncontrolled, Red, Amber, Green }
Spawn_Runtime :: struct { next: [dynamic]int, elapsed: f64, pending, admitted: int }
World :: struct {
    nodes: [dynamic]Node,
    roads: [dynamic]Road,
    vehicles: [dynamic]Vehicle,
    time: f64,
    completed: int,
    spawned: int,
    rng: u64,
    behavior_seed: u64,
    behavior: Driver_Config,
    spawn_accumulator: f32,
    signals: [dynamic]Signal_Plan,
    spawns: [dynamic]Spawn_Point,
    requested, pending: int,
    spawn_runtime: [dynamic]Spawn_Runtime,
    stop_lines: [dynamic]f32,
    road_offsets: [dynamic]f32,
    // Deduplicated unsupported transitions; other roads continue running.
    motion_issues: [dynamic]Motion_Issue,
    fit_cache: map[Motion_Key]Motion_Fit,
    entry_sequence: u64,
}

kind_width :: proc(kind: Vehicle_Kind) -> f32 {
    switch kind {
    case .Motorcycle: return 0.75
    case .Bicycle: return 0.65
    case .Tricycle: return 1.35
    case .Car: return 1.9
    case .Bus: return 2.5
    case .Truck: return 2.4
    }
    return 1.0
}

kind_length :: proc(kind: Vehicle_Kind) -> f32 {
    switch kind {
    case .Motorcycle: return 1.9
    case .Bicycle: return 1.8
    case .Tricycle: return 2.8
    case .Car: return 4.5
    case .Bus: return 10.0
    case .Truck: return 8.0
    }
    return 3.0
}

kind_speed :: proc(kind: Vehicle_Kind) -> f32 {
    switch kind {
    case .Motorcycle: return 9.0
    case .Bicycle: return 4.5
    case .Tricycle: return 5.5
    case .Car: return 7.0
    case .Bus: return 5.5
    case .Truck: return 5.0
    }
    return 5.0
}

init :: proc(w: ^World, nodes: []Node, roads: []Road, seed: u64 = 1) {
    w^ = {}
    w.nodes = make([dynamic]Node)
    w.roads = make([dynamic]Road)
    w.vehicles = make([dynamic]Vehicle)
    w.signals = make([dynamic]Signal_Plan)
    w.spawns = make([dynamic]Spawn_Point)
    w.spawn_runtime = make([dynamic]Spawn_Runtime)
    w.stop_lines = make([dynamic]f32)
    w.road_offsets = make([dynamic]f32)
    w.motion_issues = make([dynamic]Motion_Issue)
    w.fit_cache = make(map[Motion_Key]Motion_Fit)
    for n in nodes { append(&w.nodes, n) }
    for r in roads { append(&w.roads, r) }
    w.rng = seed
    w.behavior_seed = seed
    w.behavior = default_driver_config()
}

init_demo :: proc(w: ^World, seed: u64 = 1) {
    nodes := []Node{{{0, 0}}, {{45, 0}}, {{85, 0}}, {{45, 30}}, {{85, 30}}}
    roads := []Road{
        {0, 1, 7.0, 9.0}, {1, 2, 7.0, 9.0},
        {3, 1, 5.0, 7.0}, {2, 4, 5.0, 7.0},
    }
    init(w, nodes, roads, seed)
}

destroy :: proc(w: ^World) {
    delete(w.nodes)
    delete(w.roads)
    delete(w.vehicles)
    delete(w.signals)
    delete(w.spawns)
    for &runtime in w.spawn_runtime { delete(runtime.next) }
    delete(w.spawn_runtime)
    delete(w.stop_lines)
    delete(w.road_offsets)
    delete(w.motion_issues)
    delete(w.fit_cache)
    w^ = {}
}

random_u32 :: proc(w: ^World) -> u32 {
    // Small local LCG keeps spawn choices reproducible without platform RNG state.
    w.rng = w.rng * 6364136223846793005 + 1442695040888963407
    return u32(w.rng >> 32)
}

road_length :: proc(w: ^World, road_id: int) -> f32 {
    r := w.roads[road_id]
    a, b := w.nodes[r.from].position, w.nodes[r.to].position
    dx, dy := b.x-a.x, b.y-a.y
    return math.sqrt(dx*dx + dy*dy)
}

free_lateral :: proc(w: ^World, road_id: int, distance: f32, kind: Vehicle_Kind, lateral: f32) -> bool {
    road := w.roads[road_id]
    half := road.width * 0.5
    if abs(lateral) + kind_width(kind)*0.5 > half { return false }
    state := signal_state(w, road_id)
    if (state == .Red || state == .Amber) && distance + kind_length(kind)*0.5 > w.stop_lines[road_id] { return false }
    if !body_clear(w, Vehicle{road = road_id, distance = distance, lateral = lateral, kind = kind}) { return false }
    for other in w.vehicles {
        if other.motion.active || other.road != road_id || abs(other.distance-distance) > (kind_length(kind)+kind_length(other.kind))*0.5 + 1.2 { continue }
        if abs(other.lateral-lateral) < (kind_width(kind)+kind_width(other.kind))*0.5 + 0.18 { return false }
    }
    return true
}

fitting_lateral :: proc(w: ^World, road_id: int, kind: Vehicle_Kind, desired: f32) -> (f32, bool) {
    limit := max(0, w.roads[road_id].width*0.5-kind_width(kind)*0.5)
    center := clamp(desired, -limit, limit)
    if free_lateral(w, road_id, 0, kind, center) { return center, true }
    // Free intervals begin at a body boundary or a road edge. Test each boundary;
    // O(n^2) per transition, O(n^3) worst-case per tick; replace with sorted intervals.
    for other in w.vehicles {
        if other.motion.active || other.road != road_id || abs(other.distance) > (kind_length(kind)+kind_length(other.kind))*0.5 + 1.2 { continue }
        gap := (kind_width(kind)+kind_width(other.kind))*0.5 + 0.18
        left, right := other.lateral-gap-0.00001, other.lateral+gap+0.00001
        if left >= -limit && left <= limit && free_lateral(w, road_id, 0, kind, left) { return left, true }
        if right >= -limit && right <= limit && free_lateral(w, road_id, 0, kind, right) { return right, true }
    }
    if free_lateral(w, road_id, 0, kind, -limit) { return -limit, true }
    if free_lateral(w, road_id, 0, kind, limit) { return limit, true }
    return 0, false
}

try_spawn :: proc(w: ^World) {
    if len(w.roads) == 0 || len(w.vehicles) >= 120 { return }
    // Synthetic demand mix: 65% motorcycles, 15% cars, 8% bicycles, 5% buses,
    // 4% trucks, and 3% tricycles; balancing weights, not local calibration.
    draw := random_u32(w) % 100
    kind := Vehicle_Kind(.Motorcycle)
    switch {
    case draw < 65: kind = .Motorcycle
    case draw < 80: kind = .Car
    case draw < 88: kind = .Bicycle
    case draw < 93: kind = .Bus
    case draw < 97: kind = .Truck
    case: kind = .Tricycle
    }
    road_id := int(random_u32(w) % u32(len(w.roads)))
    // Sample a lateral offset; reject spawn if its body does not fit or is occupied.
    road := w.roads[road_id]
    step := f32(random_u32(w) % 3) * 0.85
    sign := f32(1)
    if random_u32(w) % 2 == 0 { sign = -1 }
    lateral := step * sign
    candidate := Vehicle{road = road_id, lateral = lateral, kind = kind, spawn_index = -1}
    structural := source_problem(w, candidate)
    if structural != .None { report_motion_issue(w, candidate, successor(w, candidate), structural); return }
    if !free_lateral(w, road_id, 0, kind, lateral) { return }
    if !source_supported(w, candidate) { return }
    candidate.id, candidate.preferred_lateral = u64(w.spawned+1), lateral
    candidate.driver, _ = make_driver(w.behavior, w.behavior_seed, candidate.id, kind, spawn_persona(w.behavior, w.behavior_seed, candidate.id))
    append(&w.vehicles, candidate)
    w.spawned += 1
}

next_road :: proc(w: ^World, current: int) -> int {
    node := w.roads[current].to
    // First non-reversing link in input order, else U-turn: no destination or cycle detection.
    fallback := -1
    for i in 0..<len(w.roads) {
        r := w.roads[i]
        if r.from == node {
            if fallback < 0 { fallback = i }
            if r.to != w.roads[current].from { return i }
        }
    }
    return fallback
}

// Seeded successor among non-reversing exits. wander=0 keeps next_road's first choice.
choose_next_road :: proc(w: ^World, v: Vehicle) -> int {
    node := w.roads[v.road].to
    fallback := -1
    options: [8]int
    n := 0
    for i in 0..<len(w.roads) {
        r := w.roads[i]
        if r.from != node { continue }
        if fallback < 0 { fallback = i }
        if r.to == w.roads[v.road].from { continue }
        if v.driver.wander <= 0 { return i }
        if n < len(options) { options[n] = i; n += 1 }
    }
    if n == 0 { return fallback }
    if n == 1 { return options[0] }
    if driver_sample(w.behavior_seed, v.id, 0x9b05688c2b3e6c1d ~ u64(v.road)) >= v.driver.wander { return options[0] }
    pick := int(driver_sample(w.behavior_seed, v.id, 0x27bb2ee687b0b0fd ~ u64(v.road))*f32(n))
    if pick >= n { pick = n-1 }
    return options[pick]
}

step :: proc(w: ^World, dt: f32) {
    if dt <= 0 || !finite(dt) || len(w.roads) == 0 { return }
    if len(w.stop_lines) != len(w.roads) { cache_geometry(w) }
    remaining := dt
    for remaining > 0 {
        tick := min(remaining, 0.05)
        step_tick(w, tick)
        remaining -= tick
    }
}

step_tick :: proc(w: ^World, dt: f32) {
    w.time += f64(dt)
    if len(w.spawns) > 0 {
        scheduled_spawns(w, dt)
    } else {
        w.spawn_accumulator += dt
        for w.spawn_accumulator >= 0.8 {
            w.spawn_accumulator -= 0.8
            try_spawn(w)
        }
    }
    // Stable array arbitration; O(n^2) following/body scans remain capped at 120.
    i := 0
    for i < len(w.vehicles) {
        v := w.vehicles[i]
        headway, acceleration := f32(0.8), f32(2)
        if w.behavior.enabled { headway, acceleration = v.driver.traits.headway, v.driver.traits.acceleration }
        if v.motion.active && connector_is_left(v.motion) && v.driver.actor.junction_yielded { acceleration = max(acceleration, 3.5) }
        following: Following_Observation
        adjustment := following_acceleration(w, v, i, headway, acceleration, &following)
        observation := observe_driver(w, i, following)
        driver_receive(&w.vehicles[i].driver, Driver_Event{kind = .Tick, time = w.time, dt = dt, observation = observation})
        intent := w.vehicles[i].driver.actor.intent
        plan: Driver_Feedback
        switch intent {
        case .Recover: _ = try_local_bypass(w, i, &plan)
        case .Pass: try_overtake(w, i, &plan)
        case .Continue:
        }
        if intent != .Continue {
            observation = observe_driver(w, i, following)
            driver_receive(&w.vehicles[i].driver, Driver_Event{kind = .Maneuver_Result, time = w.time, observation = observation, feedback = plan})
        }
        v = w.vehicles[i]
        target := min(w.roads[v.road].speed_limit, kind_speed(v.kind))
        if v.motion.active && v.motion.exit >= 0 { target = min(target, w.roads[v.motion.exit].speed_limit) }
        if w.behavior.enabled { target *= v.driver.traits.speed_factor }
        // These are targets, not hard limits: finite braking can leave an
        // entering vehicle temporarily above a curve or receiving-road target.
        if v.motion.active {
            _, straight := straight_direction(v.motion)
            if !straight { target = min(target, 4) }
            if v.motion.bypass { target = min(target, 2) }
            // After yielding in the box, complete a left at a committed pace
            // rather than crawling the rest of the curve at the 4 m/s target.
            if connector_is_left(v.motion) && v.driver.actor.state == .Traverse && v.driver.actor.junction_yielded {
                cap := min(w.roads[v.road].speed_limit, kind_speed(v.kind))
                if v.motion.exit >= 0 { cap = min(cap, w.roads[v.motion.exit].speed_limit) }
                target = min(cap, 6.5)
                if w.behavior.enabled { target *= v.driver.traits.speed_factor }
            }
        }
        if v.motion.active && v.motion.steering_count > 0 {
            for k in int(v.motion.progress/STEERING_STEP)..<v.motion.steering_count {
                curvature := abs(v.motion.steering[k])
                if curvature == 0 { continue }
                // Synthetic 1rad/s manoeuvring yaw limit; brake before, rather
                // than at, a tighter action. Physical movement checks remain.
                corner_speed := min(STEERING_SPEED, STEERING_YAW_RATE/curvature)
                distance := max(0, f32(k)*STEERING_STEP-v.motion.progress)
                target = min(target, math.sqrt(corner_speed*corner_speed+8*distance))
            }
        }
        if v.motion.active && v.motion.partial {
            target = min(target, math.sqrt(max(0, 8*(v.motion.length-v.motion.progress))))
        }
        if !v.motion.active {
            // Brake towards a slower exit or a known change of heading before
            // commitment. Parallel lateral shifts may still fit as curves;
            // late admissions and finite ticks can also exceed these targets.
            exit := successor(w, v)
            if exit >= 0 {
                exit_target := min(w.roads[exit].speed_limit, kind_speed(v.kind))
                if w.behavior.enabled { exit_target *= v.driver.traits.speed_factor }
                _, incoming := vehicle_pose(w, v)
                _, outgoing := vehicle_pose(w, Vehicle{road = exit})
                if incoming.x*outgoing.y-incoming.y*outgoing.x != 0 || incoming.x*outgoing.x+incoming.y*outgoing.y <= 0 {
                    exit_target = min(exit_target, 4)
                }
                distance := max(0, entry_distance(w, v.road, v.kind)-v.distance)
                target = min(target, math.sqrt(exit_target*exit_target+8*distance))
            }
            stop := entry_distance(w, v.road, v.kind)
            state := signal_state(w, v.road)
            if (state == .Red || state == .Amber) && v.distance <= stop+0.0001 {
                target = min(target, math.sqrt(max(0, 8*(stop-v.distance))))
            }
        }
        speed := v.speed
        if v.motion.active && connector_is_left(v.motion) && v.driver.actor.state == .Traverse && v.driver.actor.junction_yielded {
            acceleration = max(acceleration, 3.5)
        }
        if speed < target { speed = min(target, speed+acceleration*dt) } else { speed = max(target, speed-4*dt) }
        // Reuse perception unless an accepted maneuver changed the actual guide.
        if plan.accepted { adjustment = following_acceleration(w, v, i, headway, acceleration, &following) }
        speed = max(0, min(speed, v.speed+adjustment*dt))
        budget := speed*dt
        // Following and execution both use actual world-space body clearance;
        // a second road/lateral projection would stop clear parallel traffic.
        // Forecast this tick's intended speed, not an invented minimum pace.
        // Movement may still be curtailed by emergency physical checks.
        w.vehicles[i].speed = speed
        result: Driver_Feedback
        travelled, finished := move_vehicle(w, i, budget, &result)
        if result.reason == .None && travelled <= 0.000001 {
            if observation.signal_hold { result.reason = .Signal
            } else if following.found && budget <= 0.000001 { result.reason, result.blocker = .Following, following.leader }
        }
        result.moved, result.finished = travelled, finished
        observation = observe_driver(w, i, following)
        driver_receive(&w.vehicles[i].driver, Driver_Event{kind = .Motion_Result, time = w.time, dt = dt, observation = observation, feedback = result})
        if finished {
            w.completed += 1
            ordered_remove(&w.vehicles, i)
            continue
        }
        w.vehicles[i].speed = min(speed, travelled/dt)
        i += 1
    }
}
