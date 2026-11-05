package sim

import "core:testing"

@(test)
test_actor_junction_yield_requires_continuous_no_progress :: proc(t: ^testing.T) {
    d: Driver
    o := Driver_Observation{motion = true, can_recover = true, speed = 1}
    for tick in 0..<6 {
        time := f64(tick)*0.1
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = time, dt = 0.05, observation = o,
            feedback = {reason = .Predicted_Conflict, blocker = 2}})
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = time+0.05, dt = 0.05, observation = o,
            feedback = {moved = 0.05}})
    }
    testing.expect(t, !d.actor.junction_yielded && d.actor.junction_wait == 0,
        "interleaved movement must not add separate brief stops into a committed yield")
    for tick in 0..<5 {
        driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 1+f64(tick)*0.05, dt = 0.05, observation = o,
            feedback = {reason = .Predicted_Conflict, blocker = 2}})
    }
    testing.expect(t, d.actor.junction_yielded && d.actor.junction_wait >= DRIVER_STALL_SECONDS)
    driver_receive(&d, Driver_Event{kind = .Motion_Result, time = 1.3, dt = 0.05, observation = o,
        feedback = {moved = 0.05}})
    testing.expect(t, d.actor.junction_yielded && d.actor.junction_wait == 0,
        "after a genuine continuous yield, resumed movement retains the committed-turn decision")
}
