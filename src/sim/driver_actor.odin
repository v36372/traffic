package sim

// One native, value-owned actor per trip. Leaves have explicit shared parents;
// this is not a general statechart runtime or a thread per vehicle.
Driver_Region :: enum { Route, Maneuver, Recovery, Done }
Driver_State :: enum { Cruise, Follow_Queue, Wait_Signal, Wait_Entry, Pass, Traverse, Yield_Junction, Plan, Execute, Retry, Completed }
Driver_Intent :: enum { Continue, Pass, Recover }
Driver_Event_Kind :: enum { Tick, Maneuver_Result, Motion_Result }
Driver_Actor :: struct {
    initialized: bool,
    state: Driver_State,
    intent: Driver_Intent,
    since: f64,
    last, plan: Driver_Feedback,
    attempts: u64,
    junction_wait: f32,
    junction_yielded, filtering: bool,
}
Driver_Observation :: struct {
    motion, lane, bypass, partial, near_end, filtering: bool,
    signal_hold, at_entry, following, can_recover: bool,
    speed: f32,
    leader: u64,
}
Driver_Event :: struct {
    kind: Driver_Event_Kind,
    time: f64,
    dt: f32,
    observation: Driver_Observation,
    feedback: Driver_Feedback,
}

DRIVER_RETRY_SECONDS :: f64(0.5)
DRIVER_STALL_SECONDS :: f32(0.25)

driver_state_parent :: proc(state: Driver_State) -> Driver_Region {
    switch state {
    case .Cruise, .Follow_Queue, .Wait_Signal, .Wait_Entry: return .Route
    case .Pass, .Traverse, .Yield_Junction: return .Maneuver
    case .Plan, .Execute, .Retry: return .Recovery
    case .Completed: return .Done
    }
    unreachable()
}

driver_state_path :: proc(state: Driver_State) -> string {
    switch state {
    case .Cruise: return "Driving.Route.Cruise"
    case .Follow_Queue: return "Driving.Route.FollowQueue"
    case .Wait_Signal: return "Driving.Route.WaitSignal"
    case .Wait_Entry: return "Driving.Route.WaitEntry"
    case .Pass: return "Driving.Maneuver.Pass"
    case .Traverse: return "Driving.Maneuver.Traverse"
    case .Yield_Junction: return "Driving.Maneuver.YieldJunction"
    case .Plan: return "Driving.Recovery.Plan"
    case .Execute: return "Driving.Recovery.Execute"
    case .Retry: return "Driving.Recovery.RetryWait"
    case .Completed: return "Completed"
    }
    unreachable()
}

// Read-only physical facts at this driver's serial turn. The actual following
// controller supplies the leader; road/lateral labels cannot invent one here.
observe_driver :: proc(w: ^World, i: int, following: Following_Observation) -> Driver_Observation {
    v := w.vehicles[i]
    c := v.motion
    stop := entry_distance(w, v.road, v.kind)
    state := signal_state(w, v.road)
    o := Driver_Observation{motion = c.active, lane = v.lane.active, bypass = c.bypass,
        partial = c.partial, filtering = v.lane.active && v.lane.length > 0 && v.lane.length < LANE_CHANGE_LENGTH,
        speed = v.speed, following = following.found, leader = following.leader,
        at_entry = !c.active && v.distance >= stop-0.0001}
    o.signal_hold = !c.active && (state == .Red || state == .Amber) && v.speed < 0.1 && v.distance >= stop-0.2 && v.distance <= stop+0.0001
    // A queue far upstream can still use a same-link corridor on red; only
    // the actual mandatory bar hold suppresses recovery pressure.
    if c.active {
        o.near_end = (c.partial || v.speed < 0.5) && c.length-c.progress <= max(0.5, v.speed*v.speed/8+v.speed*0.05)
        o.can_recover = c.exit >= 0 && c.progress >= c.lead
    } else {
        o.can_recover = stop-v.distance > 2*kind_length(v.kind)+STEERING_STEP && !o.signal_hold
    }
    return o
}

// Also used once when importing historical native captures. No pose, route,
// lane, guide or priority is changed when decision ownership is initialized.
initialize_driver_actor :: proc(d: ^Driver, o: Driver_Observation, time: f64) {
    if d.actor.initialized { return }
    d.actor = Driver_Actor{initialized = true, since = time, filtering = o.filtering}
    if o.motion {
        d.actor.state = .Traverse
        if o.bypass { d.actor.state = .Execute }
        if d.waited >= DRIVER_STALL_SECONDS { d.actor.junction_yielded = true }
    } else if o.lane { d.actor.state = .Pass
    } else if o.signal_hold { d.actor.state = .Wait_Signal
    } else if o.following { d.actor.state = .Follow_Queue }
    publish_driver_mode(d)
}

// Only the actor publishes this legacy inspector projection. Mode is never a
// decision input; keeping it retains old native cases and existing consumers.
publish_driver_mode :: proc(d: ^Driver) {
    switch d.actor.state {
    case .Cruise, .Completed: d.mode = .Cruising
    case .Follow_Queue:
        d.mode = .Following
        if d.patience > 0 && d.waited >= d.patience { d.mode = .Seeking }
    case .Wait_Signal: d.mode = .Stopped_Signal
    case .Wait_Entry: d.mode = .Following_Junction
    case .Pass:
        d.mode = .Passing
        if d.actor.filtering { d.mode = .Filtering }
    case .Traverse: d.mode = .Junction
    case .Yield_Junction: d.mode = .Following_Junction
    case .Plan, .Execute, .Retry: d.mode = .Avoiding
    }
}

driver_transition :: proc(d: ^Driver, state: Driver_State, time: f64) {
    if d.actor.state != state { d.actor.state, d.actor.since = state, time }
}

driver_physical_state :: proc(o: Driver_Observation, feedback: Driver_Feedback) -> Driver_State {
    if o.motion {
        if o.bypass { return .Execute }
        if feedback.reason != .None && feedback.moved == 0 { return .Yield_Junction }
        return .Traverse
    }
    if o.lane { return .Pass }
    if o.signal_hold { return .Wait_Signal }
    if o.at_entry && feedback.reason != .None { return .Wait_Entry }
    if o.following { return .Follow_Queue }
    return .Cruise
}

// A fixed mailbox protocol: Tick -> optional Maneuver_Result -> Motion_Result.
// Process one event to completion; result events never emit another action.
// There is at most one bounded maneuver request per driver per native tick.
driver_receive :: proc(d: ^Driver, event: Driver_Event) {
    o := event.observation
    initialize_driver_actor(d, o, event.time)
    if d.actor.state == .Completed { return }
    d.actor.filtering = o.filtering
    switch event.kind {
    case .Tick:
        d.actor.intent = .Continue
        // Shared Driving guard: no recovery pressure from a mandatory signal
        // hold. Already-committed junction drivers are not new red entrants.
        if o.signal_hold {
            d.waited = 0
            driver_transition(d, .Wait_Signal, event.time)
        } else {
            stalled := d.motion_blocked || d.waited >= DRIVER_STALL_SECONDS || o.near_end
            if o.can_recover && (o.speed < 0.5 || o.near_end) && stalled {
                // Recovery parent handles the deadline for every child. An
                // executing, moving prefix remains stable until it needs help.
                if event.time >= d.replan_after {
                    d.actor.intent = .Recover
                    d.actor.attempts += 1
                    d.replan_after = event.time+DRIVER_RETRY_SECONDS
                    driver_transition(d, .Plan, event.time)
                } else {
                    driver_transition(d, .Retry, event.time)
                }
            } else {
                driver_transition(d, driver_physical_state(o, d.actor.last), event.time)
                if !o.motion && !o.lane && o.following { d.actor.intent = .Pass }
            }
        }
    case .Maneuver_Result:
        d.actor.plan = event.feedback
        if event.feedback.accepted {
            driver_transition(d, driver_physical_state(o, {}), event.time)
        } else if d.actor.intent == .Recover {
            driver_transition(d, .Retry, event.time)
        }
    case .Motion_Result:
        result := event.feedback
        d.actor.last = result
        d.motion_blocked = result.reason == .Body || result.reason == .Predicted_Conflict || result.reason == .Rear_Gap || result.reason == .Pavement
        if result.finished {
            driver_transition(d, .Completed, event.time)
        } else {
            if o.signal_hold || result.moved > 0.000001 { d.waited = 0 } else { d.waited += event.dt }
            if o.motion && !o.bypass {
                if result.moved <= 0.000001 && !o.signal_hold { d.actor.junction_wait += event.dt }
                if d.actor.junction_wait >= DRIVER_STALL_SECONDS { d.actor.junction_yielded = true }
            } else if !o.motion {
                d.actor.junction_wait, d.actor.junction_yielded = 0, false
            }
            // A failed search remains RetryWait while stopped; physical facts
            // can finish a maneuver, but a mode poll cannot erase its history.
            waiting_retry := driver_state_parent(d.actor.state) == .Recovery && !o.bypass && result.moved == 0
            if !waiting_retry { driver_transition(d, driver_physical_state(o, result), event.time) }
        }
    }
    publish_driver_mode(d)
}
