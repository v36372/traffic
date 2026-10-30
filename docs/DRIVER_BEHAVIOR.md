# Driver variation and rule-breaking — proposed, not implemented

User request: configurable motorcycle behavior such as accelerating to catch a light, entering on amber and moving sideways to get ahead. This extends the authenticity goal; it does not excuse placement, collision or path-fitting bugs.

## Five tuning inputs

| Input | Meaning |
|---|---|
| Desired speed distribution | Individual preferred speed relative to the road limit, bounded by vehicle acceleration/braking and available space. |
| Gap acceptance | Preferred following time and lateral clearance. More assertive riders accept smaller gaps; physical body clearance is never randomized away. |
| Signal compliance | Willingness to attempt a prohibited entry. Decide once per relevant approach/phase event, not with a new coin flip every frame. |
| Filtering tendency | Willingness to seek a usable lateral corridor past a queue. Distinguish passing within the carriageway from crossing a centerline into opposing traffic. |
| Patience | Waiting time before increasing pressure to seek a gap or attempt a violation; bounded and reset after the obstruction clears. |

Use per-vehicle-class distributions, not identical agents or a single global chaos multiplier. Sample persistent individual traits from a separate seeded behavior stream (or deterministic keyed samples), independently of appearance and arrival randomness. Stable IDs already exist. Same map, configuration and seed must replay the same decisions.

Numbers and presets will be explicitly synthetic until calibrated against observations. Never imply all Vietnamese riders or all motorcycles behave identically.

## Decisions, not random motion

Separate ordinary actions, deliberate infractions and simulation defects. Clearing a crossing already entered before the signal change is not the same event as entering after it. Amber decisions depend on distance, speed, reaction/braking ability and willingness to proceed. Acceleration to catch a light must obey dynamics and should not rely on perfect knowledge of an unseen signal clock.

Filtering needs a continuous, validated lateral path and merge/gap checks. Do not change a vehicle's lateral coordinate instantaneously or ignore opposite-direction vehicles. Current fixed lateral source slots are **not** filtering. Wrong-way encroachment, red entry and sidewalk use must be separately identifiable; none is silently implied by an aggressiveness slider.

Rule-breaking does not disable body overlap checks, swept movement validation or pavement geometry. Crash mechanics are a separate explicit feature, not numerical overlap presented as realism. If conservative conflict reservations prevent an attempted unsafe entry, record the attempt as blocked rather than pretend a violation happened.

## Verified rule reference

Vietnam Law36/2024/QH15, Article11(4)(b), as published on the [government policy portal](https://xaydungchinhsach.chinhphu.vn/toan-van-luat-trat-tu-an-toan-giao-thong-duong-bo-119240909105718285.htm), says steady yellow requires stopping before the stop line; vehicles already on or past it when yellow appears may continue. Flashing yellow is a separate proceed-with-caution/yield rule. Verified from the published text on2026-10-30. Article11 also explicitly allows signals with or without a countdown display.

Therefore do not automatically count every vehicle moving during amber as an infraction, or assume every rider knows the exact remaining green time. Braking feasibility is a physical decision input, not an invented blanket legal exception. Current simulator has no flashing-yellow mode; do not imply otherwise.

## Implementation order

1. Finish the current geometry/crossing regressions and keep a fully compliant deterministic baseline.
2. Add speed/headway variation and reaction/amber decisions with auditable counters and XML settings.
3. Add continuous motorcycle filtering, then separately controlled prohibited maneuvers once their movement/conflict rules exist.

Do not accept schema settings for behavior that has no runtime implementation. Police presence/enforcement can later modify decision likelihood; congestion and delay modify patience. Those management links are not implemented yet.

## Checks and observability

Retain replay and physical-clearance tests. Add per-driver trait persistence, safe minimum spacing, zero-violation compliant mode, once-per-event decisions, distinct amber entry versus committed clearance, and counters for attempted/completed/blocked violations. Compare compliant and varied-driver runs under the same map, demand and seed. Visual review must distinguish intentional, counted behavior from actual bugs.
