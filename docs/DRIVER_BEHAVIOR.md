# Driver variation and rule-breaking

Bounded, synthetic speed/headway traits are opt-in per class via `<driver>` as documented in [MAP_BUILDER.md](MAP_BUILDER.md). Stable driver IDs and the world seed key independent speed/headway, passing-side, aggressiveness, turn-bias and wander samples without consuming arrival RNG; traits persist through a trip. A class with a nontrivial speed or headway range also gets an independent acceleration sample from 1.4–2.6 m/s². Identical bounds retain the baseline 2 m/s². Speed factors never exceed the compliant baseline, following preference never removes physical clearance checks, and signal rules are unchanged. Reset retains settings.

When any class has a nontrivial range, `make_driver` also samples a persistent aggressiveness in a persona band (cautious 0.05–0.35, balanced 0.25–0.70, assertive 0.60–1.00), a patience delay, a turn-bias and a wander probability. Motorcycles use those traits to squeeze past slower leaders, pick different receiving bands and, on synthetic-demand maps, choose among non-reversing exits. Other classes keep the sampled values but only two-wheelers apply turn-bias, shorter filters and reduced gap horizons. The maneuver stays within the directed carriageway and finishes before the stop bar; it is not opposing-lane filtering or a red-light violation. Body checks are sampled, not continuous swept-volume certification. Reaction, amber decisions and deliberate infractions remain unimplemented. All values are synthetic, not measured driving behavior.

User request: configurable motorcycle behavior such as accelerating to catch a light, entering on amber and moving sideways to get ahead. This extends the authenticity goal; it does not excuse placement, collision or path-fitting bugs.

## Reusable driver factory

`sim.make_driver(config, seed, id, kind, persona)` returns an independent `Driver` value and an `ok` flag. Start with `sim.default_driver_config()`, then enable and set the desired class ranges. IDs must be nonzero; invalid vehicle kinds, personas or enabled ranges return `ok=false`. A disabled configuration uses baseline traits. Assign the result to a `Vehicle` of the same kind before adding it to a world configured with those ranges.

```odin
config := sim.default_driver_config()
config.enabled = true
config.classes[int(sim.Vehicle_Kind.Motorcycle)] = {0.65, 1, 0.5, 1.5}
driver, ok := sim.make_driver(config, 42, 1, .Motorcycle, .Assertive)
// Configure the world before starting it; then assign driver to a motorcycle.
```

Each driver owns persistent traits and a [native actor/statechart](DRIVER_ACTORS.md). Tick and execution-result events drive explicit Route, Maneuver and Recovery states and `Continue`, `Pass` or `Recover` intents. The old `Cruising`/`Following`/`Passing`/`Avoiding` modes are actor-published inspector projections, not a separately recomputed decision machine. Actual no-progress timing, execution veto reasons and bounded retry deadlines control recovery; a mandatory signal hold does not build recovery pressure. Intersection paths are steering guides, not reservations. Uncontrolled left turns start as a pavement-fitted diagonal from the entry; after the driver has yielded, the remaining curve uses a higher speed target so the turn is committed. Local bypasses instead use the slower target described below. Drivers use a synthetic rolling prediction of nearby bodies (1.5 s at zero aggressiveness, down to about 0.95 s for a fully assertive motorcycle), yielding to earlier entrants at imminent conflicts. Turns and crossing streams may occupy a junction together when local time/space gaps permit. No receiving strip or outgoing ray is owned, and a leader need not wait for followers before turning at its next junction.

The shared actor policy uses persona, vehicle kind and the sampled aggressiveness: assertive, balanced and cautious drivers start from 0.5, 1 and 2 m/s of needed speed advantage before a pass; buses and trucks require another 0.5 m/s; two-wheelers then subtract up to 0.7 m/s of that threshold in proportion to aggressiveness. After a rider has waited in a queue for their personal patience (about 0.45–3.9 s), a motorcycle will take almost any slower leader and may use a 5–6 m squeeze instead of the 12 m pass. At junction entry, all vehicles may try the next-nearest fitted receiving band if the preferred trajectory is occupied; every option still uses the same physical, signal and clearance checks. The constructor is deterministic and does not consume demand RNG. Both spawn paths create drivers through it; in a varied configuration, each spawned trip gets a deterministic synthetic persona. This is one parameterized machine, not separately authored driving models for every combination. Signal violations are not implemented.

## Steering around a stopped obstruction

A stalled driver can enter `Avoiding` after a rejected movement, a stopped obstruction, or the end of a short steering plan. A bounded pose-space search tries straight and curved forward actions from the actual position and heading. It applies inside junctions, on ordinary links and after a vetoed lane change, independently of persona variation. It preserves the intended exit, not the old spline, waypoint heading or receiving band. Once the whole aligned body is inside its receiving strip it can continue normally; a same-link maneuver never advances the trip's route edge.

The search considers at most 2,048 expansions and 8,192 states, with up to 32 actions of 0.25m (8m ahead). Candidate actions use the unchanged body SAT margin and actual asphalt; actions are refined at 0.005m before expansion, with the same checks retained during execution. Junction departures must reach the correct receiving strip, and link maneuvers stay within their own strip. Both stop before the next entry gate. Protected crossing leads and red/amber signal approaches remain protected. A moving queue is not treated as a stationary obstacle.

A complete plan hands off at its actual receiving pose. A partial plan brakes before its endpoint, remains active without a road handoff, and replans; it does not assume the remaining route is clear. Steering speed targets account for upcoming curvature and stopping distance, using synthetic 2m/s travel and 1rad/s yaw targets with 4m/s² braking. These are controller targets, not calibrated actuator limits. Reconsideration is limited to twice per second, and working plans remain stable. Actual movement vetoes feed back into path choice, including lane-rear checks; ordinary float rounding or zero speed alone is not a failed movement.

This forward-only local search is not a calibrated steering model or citywide deadlock solver. Its horizon/state limits can miss a legal route, and genuine congestion still causes waiting.

`src/sim/avoidance_test.odin` covers mutually blocked riders, a stopped offset car on a normal link, rotation/storage order, deterministic replay, unchanged adoption pose, legal handoffs, full-body pavement/clearance, blocked controls and entry/stop-line limits. `src/replay/replay_test.odin` preserves the actual default250s crossing, near-end and receiving-queue failures, including an independent 7m witness against all120 frozen bodies. `src/replay/steering_regression_test.odin` preserves the first three temporal-audit stalls and later link/lane stalls with their surrounding traffic. These require real progress and no body/pavement violations; short free probes are explicitly not complete-route certificates.

## Following without a passing gap

The sampled 1.4–2.6 m/s² acceleration is a personal **ceiling**, not constant actual acceleration. Each tick, aligned leaders' world-space body gaps and relative speeds limit acceleration through a damped following controller. The synthetic desired gap is 0.5m plus speed × preferred headway. When passing is unavailable, followers ease off and settle near the leader's pace; they accelerate again as space opens. This applies on links and through connector boundaries, without treating frozen incoming-road distances as occupied positions.

Rolling gap prediction uses the proposed speed for this tick, including speeds below 1m/s; it no longer pretends that a slowly following rider will advance at least 1m/s. Following measures actual body contact along the guide in 0.1m samples with four contact refinements, capped at the existing 35m lookahead. An angled body's tangent projection cannot invent a leader in empty space. The redundant road/lateral movement clamp is removed; world-space movement/body checks remain authoritative. This is sampled synthetic following, not a calibrated vehicle model or prediction of future acceleration.

## Five tuning inputs

| Input | Meaning |
|---|---|
| Desired speed distribution | Individual preferred speed relative to the road limit, bounded by vehicle acceleration/braking and available space. |
| Gap acceptance | Preferred following time and lateral clearance. More assertive riders accept smaller gaps; physical body clearance is never randomized away. |
| Signal compliance | Willingness to attempt a prohibited entry. Decide once per relevant approach/phase event, not with a new coin flip every frame. |
| Filtering tendency | Willingness to seek a usable lateral corridor past a queue. Same-carriageway motorcycle squeezes and occupied-band cuts are implemented; crossing a centerline into opposing traffic is not. |
| Patience | Sampled waiting time before a stuck rider increases passing pressure. Bounded and reset after the obstruction clears. Signal violations still do not use this input. |

Use per-vehicle-class distributions, not identical agents or a single global chaos multiplier. Sample persistent individual traits from a separate seeded behavior stream (or deterministic keyed samples), independently of appearance and arrival randomness. Stable IDs already exist. Same map, configuration and seed must replay the same decisions.

Numbers and presets will be explicitly synthetic until calibrated against observations. Never imply all Vietnamese riders or all motorcycles behave identically.

## Decisions, not random motion

Separate ordinary actions, deliberate infractions and simulation defects. Clearing a crossing already entered before the signal change is not the same event as entering after it. Amber decisions depend on distance, speed, reaction/braking ability and willingness to proceed. Acceleration to catch a light must obey dynamics and should not rely on perfect knowledge of an unseen signal clock.

General queue filtering needs continuous, validated lateral paths and merge/gap checks. The implemented link-only passing maneuver is not queue filtering: it does not cross a centerline, enter a junction during the lane change or bypass a red light. Fixed lateral source slots are also **not** filtering. Wrong-way encroachment, red entry and sidewalk use must be separately identifiable; none is silently implied by an aggressiveness slider.

Rule-breaking does not disable body overlap checks, swept movement validation or pavement geometry. Crash mechanics are a separate explicit feature, not numerical overlap presented as realism. If local gap or physical clearance checks prevent an attempted unsafe entry, record the attempt as blocked rather than pretend a violation happened.

## Verified rule reference

Vietnam Law36/2024/QH15, Article11(4)(b), as published on the [government policy portal](https://xaydungchinhsach.chinhphu.vn/toan-van-luat-trat-tu-an-toan-giao-thong-duong-bo-119240909105718285.htm), says steady yellow requires stopping before the stop line; vehicles already on or past it when yellow appears may continue. Flashing yellow is a separate proceed-with-caution/yield rule. Verified from the published text on2026-10-30. Article11 also explicitly allows signals with or without a countdown display.

Therefore do not automatically count every vehicle moving during amber as an infraction, or assume every rider knows the exact remaining green time. Braking feasibility is a physical decision input, not an invented blanket legal exception. Authored signals may set `countdown="true"` so remaining seconds appear on the 3D LED module; that display does not change stop/go rules. Current simulator has no flashing-yellow mode; do not imply otherwise.

## Implementation order

1. Finish the current geometry/crossing regressions and keep a fully compliant deterministic baseline.
2. Reaction/amber decisions and auditable counters on top of the seeded speed, headway, aggressiveness and patience traits.
3. Extend same-carriageway motorcycle filtering toward continuous queue filtering, then separately controlled prohibited maneuvers once their movement/conflict rules exist.

Do not accept schema settings for behavior that has no runtime implementation. Police presence/enforcement can later modify decision likelihood; congestion and delay modify patience. Those management links are not implemented yet.

## Checks and observability

Retain replay and physical-clearance tests. Add per-driver trait persistence, safe minimum spacing, zero-violation compliant mode, once-per-event decisions, distinct amber entry versus committed clearance, and counters for attempted/completed/blocked violations. Compare compliant and varied-driver runs under the same map, demand and seed. Visual review must distinguish intentional, counted behavior from actual bugs.
