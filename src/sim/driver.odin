package sim

// Synthetic uniform distributions; speed factors never exceed the compliant
// baseline speed. Headway changes preference, not physical body clearance.
Driver_Range :: struct { speed_min, speed_max, headway_min, headway_max: f32 }
Driver_Persona :: enum { Balanced, Cautious, Assertive }
Driver_Mode :: enum { Cruising, Following, Stopped_Signal, Passing, Junction, Following_Junction, Seeking, Filtering }
Driver :: struct {
    traits: Driver_Traits,
    persona: Driver_Persona,
    kind: Vehicle_Kind,
    mode: Driver_Mode,
    side: f32,
    aggressiveness, patience, turn_bias, wander, waited: f32,
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

driver_should_pass :: proc(v: Vehicle) -> bool {
    if v.driver.mode == .Following || v.driver.mode == .Seeking { return true }
    return v.kind == .Motorcycle && v.driver.aggressiveness > 0 && v.driver.mode == .Stopped_Signal
}

motorcycle_may_cut :: proc(v: Vehicle) -> bool {
    return v.kind == .Motorcycle && v.driver.aggressiveness >= 0.55
}

// Synthetic damped gap/closing-speed control, shared by links and connectors.
// Personal acceleration is a ceiling, not a command to accelerate into a queue.
// A local tangent projection sees aligned bodies, not a claim on future paths;
// sharp turns/crossing traffic still use local gaps and actual body clearance.
// Project onto fitted path arcs before extending this to tighter-turn following.
following_acceleration :: proc(w: ^World, v: Vehicle, ignore: int, headway, preferred: f32) -> f32 {
    p, f := vehicle_pose(w, v)
    gap, lead_speed := f32(35), f32(0)
    found := false
    for other, j in w.vehicles {
        if j == ignore { continue }
        q, g := vehicle_pose(w, other)
        alignment := f.x*g.x+f.y*g.y
        if alignment < 0.8 { continue }
        dx, dy := q.x-p.x, q.y-p.y
        along := dx*f.x+dy*f.y
        if along <= 0 { continue }
        skew := abs(f.x*g.y-f.y*g.x)
        half_width := (kind_width(v.kind)+alignment*kind_width(other.kind)+skew*kind_length(other.kind))*0.5
        if abs(dx*f.y-dy*f.x) >= half_width+0.025 { continue }
        clearance := along-(kind_length(v.kind)+alignment*kind_length(other.kind)+skew*kind_width(other.kind))*0.5
        if clearance >= gap { continue }
        gap, lead_speed, found = clearance, other.speed*alignment, true
    }
    if !found { return preferred }
    // 0.5m standstill space and >=0.4s response are synthetic preferences.
    response := max(0.4, headway)
    desired_gap := 0.5+v.speed*response
    adjustment := 2*(lead_speed-v.speed)/response+(gap-desired_gap)/(response*response)
    return clamp(adjustment, -4, preferred)
}

// Recompute from physical observations; maneuver state has priority over following.
update_driver_mode :: proc(w: ^World, i: int) {
    v := w.vehicles[i]
    mode := Driver_Mode.Cruising
    if v.motion.active {
        mode = .Junction
        if !local_gap_clear(w, v, i, v.speed, gap_horizon(v.driver)) { mode = .Following_Junction }
    } else if v.lane.active {
        mode = .Passing
        if v.lane.length > 0 && v.lane.length < LANE_CHANGE_LENGTH { mode = .Filtering }
    } else if (signal_state(w, v.road) == .Red || signal_state(w, v.road) == .Amber) && v.speed < 0.1 && v.distance >= entry_distance(w, v.road, v.kind)-0.2 { mode = .Stopped_Signal
    } else {
        for lead, j in w.vehicles {
            if i != j && lead.road == v.road && !lead.motion.active && lead.distance > v.distance && lead.distance-v.distance < 35 && abs(lead.lateral-v.lateral) < (kind_width(lead.kind)+kind_width(v.kind))*0.5+0.18 {
                mode = .Following
                if v.driver.patience > 0 && v.driver.waited >= v.driver.patience { mode = .Seeking }
                break
            }
        }
    }
    w.vehicles[i].driver.mode = mode
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
