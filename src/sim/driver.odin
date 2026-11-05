package sim
import "core:math"

// Synthetic uniform distributions; speed factors never exceed the compliant
// baseline speed. Headway changes preference, not physical body clearance.
Driver_Range :: struct { speed_min, speed_max, headway_min, headway_max: f32 }
Driver_Persona :: enum { Balanced, Cautious, Assertive }
Driver_Mode :: enum { Cruising, Following, Stopped_Signal, Passing, Junction, Following_Junction, Seeking, Filtering, Avoiding }
Driver :: struct {
    traits: Driver_Traits,
    persona: Driver_Persona,
    kind: Vehicle_Kind,
    mode: Driver_Mode,
    side: f32,
    aggressiveness, patience, turn_bias, wander, waited: f32,
    // Actor-owned no-progress timer (waited), retry deadline and published
    // legacy fields are retained for historical native replay captures.
    replan_after: f64,
    motion_blocked: bool,
    actor: Driver_Actor,
}
Driver_Config :: struct {
    enabled: bool,
    classes: [6]Driver_Range,
}
Driver_Traits :: struct { speed_factor, headway, acceleration: f32 }

default_driver_config :: proc() -> Driver_Config {
    config: Driver_Config
    for &r in config.classes { r = {1, 1, 0.8, 0.8} }
    return config
}

// Configure only before a run; invalid input leaves the previous settings intact.
configure_drivers :: proc(w: ^World, config: Driver_Config) -> bool {
    if len(w.vehicles) != 0 || w.time != 0 || w.spawned != 0 || w.completed != 0 || w.requested != 0 { return false }
    if !valid_driver_config(config) { return false }
    w.behavior = config
    return true
}

valid_driver_config :: proc(config: Driver_Config) -> bool {
    if !config.enabled { return true }
    for r in config.classes {
        if !finite(r.speed_min) || !finite(r.speed_max) || !finite(r.headway_min) || !finite(r.headway_max) || r.speed_min < 0.5 || r.speed_max > 1 || r.speed_min > r.speed_max || r.headway_min < 0.4 || r.headway_max > 2 || r.headway_min > r.headway_max { return false }
    }
    return true
}

behavior_varied :: proc(config: Driver_Config) -> bool {
    if !config.enabled { return false }
    for r in config.classes {
        if r.speed_min != r.speed_max || r.headway_min != r.headway_max { return true }
    }
    return false
}

// A value-owned trip: no world or shared RNG is retained. IDs must be nonzero.
make_driver :: proc(config: Driver_Config, seed, id: u64, kind: Vehicle_Kind, persona: Driver_Persona) -> (Driver, bool) {
    if !valid_driver_config(config) || id == 0 || int(kind) < 0 || int(kind) > int(Vehicle_Kind.Tricycle) || int(persona) < 0 || int(persona) > int(Driver_Persona.Assertive) { return {}, false }
    d := Driver{persona = persona, kind = kind, side = -1, traits = {1, 0.8, 2}}
    if driver_sample(seed, id, 0x082efa98ec4e6c89) >= 0.5 { d.side = 1 }
    if config.enabled {
        r := config.classes[int(kind)]
        acceleration: f32 = 2
        if r.speed_min != r.speed_max || r.headway_min != r.headway_max { acceleration = 1.4+1.2*driver_sample(seed, id, 0xa4093822299f31d0) }
        d.traits.speed_factor = r.speed_min+(r.speed_max-r.speed_min)*driver_sample(seed, id, 0x243f6a8885a308d3)
        d.traits.headway = r.headway_min+(r.headway_max-r.headway_min)*driver_sample(seed, id, 0x13198a2e03707344)
        d.traits.acceleration = acceleration
        if behavior_varied(config) {
            lo, hi: f32 = 0.25, 0.7
            switch persona {
            case .Cautious: lo, hi = 0.05, 0.35
            case .Assertive: lo, hi = 0.6, 1
            case .Balanced:
            }
            d.aggressiveness = lo+(hi-lo)*driver_sample(seed, id, 0xbe54646cf34e90c6)
            d.patience = 0.45+(1-d.aggressiveness)*3.4
            d.turn_bias = (driver_sample(seed, id, 0x3c6ef372fe94f82b)*2-1)*(0.35+0.65*d.aggressiveness)
            if kind == .Motorcycle { d.wander = 0.08+0.42*d.aggressiveness }
        }
    }
    return d, true
}

passing_advantage :: proc(d: Driver) -> f32 {
    threshold: f32 = 1
    switch d.persona {
    case .Cautious: threshold = 2
    case .Assertive: threshold = 0.5
    case .Balanced:
    }
    if d.kind == .Bus || d.kind == .Truck { threshold += 0.5 }
    if (d.kind == .Motorcycle || d.kind == .Bicycle) && d.aggressiveness > 0 {
        threshold -= 0.7*d.aggressiveness
        threshold = max(threshold, 0.05)
    }
    if d.patience > 0 && d.waited >= d.patience {
        threshold *= 0.5
        if d.kind == .Motorcycle { threshold = min(threshold, 0.12) }
    }
    return threshold
}

gap_horizon :: proc(d: Driver) -> f32 {
    if d.aggressiveness <= 0 || (d.kind != .Motorcycle && d.kind != .Bicycle) { return 1.5 }
    return 1.5-0.55*d.aggressiveness
}

pass_length :: proc(v: Vehicle) -> f32 {
    length := LANE_CHANGE_LENGTH
    if v.kind == .Motorcycle && v.driver.aggressiveness > 0 {
        length = 12-5.5*v.driver.aggressiveness
        if v.driver.patience > 0 && v.driver.waited >= v.driver.patience { length = min(length, 6) }
        length = max(length, 5)
    }
    return length
}

// Synthetic damped gap/closing-speed control, shared by links and connectors.
// Personal acceleration is a ceiling, not a command to accelerate into a queue.
// Measure contact along the actual guide, not overlapping tangent projections:
// an angled neighbor can have a nearby projected bumper without blocking us.
Following_Observation :: struct {
    found: bool,
    leader: u64,
    gap, lead_speed, desired_gap, adjustment: f32,
}

following_acceleration :: proc(w: ^World, v: Vehicle, ignore: int, headway, preferred: f32, observation: ^Following_Observation = nil) -> f32 {
    p, _ := vehicle_pose(w, v)
    response := max(0.4, headway)
    // Scan the acceleration-relevant distance, capped by the existing 35m
    // lookahead. Index path/body candidates before raising that ceiling or
    // the 120-agent limit (O(n^2 * path samples)).
    lookahead := min(35, 0.5+3*v.speed*response+max(0, preferred)*response*response)
    count := int(math.ceil(lookahead/0.1))
    poses: [352][2]Vec2 // <=35m in 0.1m samples, including the current pose.
    for k in 0..=count {
        future := forecast_vehicle(w, v, min(lookahead, f32(k)*0.1))
        poses[k][0], poses[k][1] = vehicle_pose(w, future)
    }
    gap, lead_speed := f32(35), f32(0)
    found := false
    leader: u64
    for other, j in w.vehicles {
        if j == ignore { continue }
        q, g := vehicle_pose(w, other)
        dx, dy := q.x-p.x, q.y-p.y
        reach := lookahead+(kind_length(v.kind)+kind_width(v.kind)+kind_length(other.kind)+kind_width(other.kind))*0.5+0.025
        if dx*dx+dy*dy > reach*reach { continue }
        for k in 0..=count {
            distance := min(lookahead, f32(k)*0.1)
            if distance >= gap+0.1 { break }
            point, forward := poses[k][0], poses[k][1]
            alignment := forward.x*g.x+forward.y*g.y
            if alignment < 0.8 || !body_poses_overlap(point, forward, v.kind, q, g, other.kind) { continue }
            low, high := max(0, distance-0.1), distance
            for _ in 0..<4 {
                middle := (low+high)*0.5
                trial := forecast_vehicle(w, v, middle)
                mp, mf := vehicle_pose(w, trial)
                if body_poses_overlap(mp, mf, v.kind, q, g, other.kind) { high = middle } else { low = middle }
            }
            if low < gap { gap, lead_speed, found, leader = low, other.speed*alignment, true, other.id }
            break
        }
    }
    if !found {
        if observation != nil { observation^ = {adjustment = preferred} }
        return preferred
    }
    // 0.5m standstill space and >=0.4s response are synthetic preferences.
    desired_gap := 0.5+v.speed*response
    adjustment := 2*(lead_speed-v.speed)/response+(gap-desired_gap)/(response*response)
    if observation != nil { observation^ = {found, leader, gap, lead_speed, desired_gap, clamp(adjustment, -4, preferred)} }
    return clamp(adjustment, -4, preferred)
}

// Varied configurations assign a stable synthetic persona to each trip.
spawn_persona :: proc(config: Driver_Config, seed, id: u64) -> Driver_Persona {
    if behavior_varied(config) { return Driver_Persona(int(driver_sample(seed, id, 0x452821e638d01377)*3)) }
    return .Balanced
}

// SplitMix64 finalizer; distinct trait keys do not consume the demand RNG.
driver_sample :: proc(seed, id, key: u64) -> f32 {
    x := seed ~ (id * 0x9e3779b97f4a7c15) ~ key
    x = (x ~ (x >> 30)) * 0xbf58476d1ce4e5b9
    x = (x ~ (x >> 27)) * 0x94d049bb133111eb
    x = x ~ (x >> 31)
    return f32(x >> 40) / 16777216.0
}
