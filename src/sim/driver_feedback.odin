package sim

// Execution facts, not permission to bypass a safety guard. A failed bounded
// search is deliberately distinct from a proof that no physical route exists.
Driver_Block_Reason :: enum {
    None,
    Signal,
    Following,
    Entry,
    Body,
    Predicted_Conflict,
    Rear_Gap,
    Pavement,
    Geometry,
    Protected_Approach,
    Route_Boundary,
    Search_Limit,
    No_Plan,
    Partial_End,
}

Driver_Feedback :: struct {
    attempted, accepted, finished, partial: bool,
    moved: f32,
    reason: Driver_Block_Reason,
    blocker: u64,
    problem: Motion_Problem,
}

// Optional output parameters keep read-only geometry queries and direct native
// fixture calls usable without manufacturing actor events or mutating a driver.
set_driver_feedback :: proc(out: ^Driver_Feedback, reason: Driver_Block_Reason, blocker: u64 = 0, problem: Motion_Problem = .None) {
    if out != nil {
        out.reason, out.blocker, out.problem = reason, blocker, problem
    }
}
