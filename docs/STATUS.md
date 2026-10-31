# Prototype status — 2026-10-30

**Not yet an authentic/calibrated HCMC traffic simulation.** Live acceptance evidence and remaining work: [AUTHENTICITY.md](AUTHENTICITY.md).

## Current changes — macOS arm64

- Intersection paths are now steering guides rather than exclusive node/exit reservations. Shared local gap acceptance forecasts nearby bodies over a synthetic 1.5-second horizon; earlier entrants receive tie priority only at imminent conflicts. Whole-path scans, outgoing-ray protection and leader-waits-for-followers gates are removed. Signals, pavement fitting and sampled body checks remain. This is not calibrated gap acceptance or a general deadlock guarantee; reference notes are in [HCM_REFERENCES.md](HCM_REFERENCES.md#motorcycle-flow-video-review).

- The local raylib bootstrap builds on macOS arm64. `make test && make check && make build` passed with **156 tests** (133 Odin + 23 Python). The current speed-matched-following busy capture opened and closed the viewer successfully, exporting 100 frames through 87–97 simulated seconds to `build/visual/following/`, with clip `build/visual/following.mp4`. CI configuration covers Linux x86_64 and macOS arm64 but has not yet run on GitHub.
- Prior straight-batching replay: in the 87–97 s busy showcase with synthetic preferences and passing enabled, the focused junction had up to three simultaneous connectors and 73 of 100 sampled frames had at least two. Those are historical counts, not local-gap-model measurements. Straight connectors use road/class speed targets; curved movements retain a 4 m/s target without exclusive node ownership.
- Prior ordered-turn implementation: in the default-demand seed-1 88–96 s showcase replay, up to five riders shared the 6→25 turn versus one before it. Its conservative ownership gates could deadlock short links and have now been replaced by local gaps; those counts are historical, not current-model measurements.
- A stopped follower no longer indefinitely vetoes a passing rider that already occupies its band. The current regression separates completing the pass before the 60 s startup from resuming after the next green; requiring movement during the 60–75 s red phase would be incorrect.
- At an uncontrolled showcase bend, a turning body may begin steering from its entry if the existing pavement fit passes. The previously held truck clears in the seed-1 120 s replay, with no sampled body overlaps; other unsupported bends remain possible.
- The showcase and HCMC extract enable persistent synthetic motorcycle speed/headway/acceleration preferences, seeded personas and link-only passing when a maneuver fits. A reusable driver factory and parameterized state machine are documented in [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md). OSM import reports directed reachability and uses supported numeric speed tags; see [MAP.md](MAP.md).
- Quiet and busy captures from before motorcycle preferences were enabled exported 100 frames each to `build/visual/latest-quiet/` and `build/visual/latest-busy/`; clips are `build/visual/latest-quiet.mp4` and `build/visual/latest-busy.mp4`. They remain unreviewed and do not show the current driver configuration.

## Speed-matched following verification

- Personal acceleration is now a ceiling. A shared world-space gap/closing-speed controller eases followers toward a slower leader's pace on links and connectors, then lets them accelerate when space opens. The 1m/s candidate forecast floor is removed; forecasts use this tick's proposed speed.
- `src/sim/following_test.odin` reproduced both bumper-chasing with no passing space and false halting in a 0.5m/s junction queue before the fix. Both regressions now pass in both storage/update orders, including body clearance, sustained rolling and matching the leader's speed. This is synthetic following control, not a measured/calibrated HCMC model.

## Local-gap verification — before speed-matched following

- `src/sim/local_flow_test.odin`: two crossing, three-abreast streams of 18 motorcycles drain with sampled body clearance at starting speeds 0/2/5 m/s and both storage/update orders. Separate checks cover a leader entering its next junction while its follower is still turning, and a receiving-road source using physically free space during a turn.
- `odin run tools/check_capture`: quiet/busy 87–97 s replay had at most 3/20 simultaneous active steering paths at the focused junction, with at least two advancing in 86/90 of 100 sampled frames. These counts include connector approach/tail segments; they are not center-box occupancy, measured throughput or an authenticity score.
- Directly inspected the busy contact sheet: multiple riders discharge together rather than a single exclusive turn owner. Preview: `build/visual/local-gap.mp4`; 100 source frames: `build/visual/local-gap/`. No independent authenticity review or real-world gap/acceleration calibration yet.

## Working

- Odin/raylib 3D viewer with orbit/zoom, a populated authored neighborhood, and imported HCMC roads plus 123 OSM-derived buildings. Simple low-rise, apartment, tower and landmark forms; explicit source heights preserved.
- XML map compiler for streets/buildings, signal controllers, and typed scheduled origins/destinations. Five authored examples; the showcase now has 11 sources including an explicit visible left-turn route.
- Shared junction geometry drives pavement, crossing paint, stop bars, pole placement and vehicle constraints. Continuous simulation-owned lead/curve/tail motion replaces endpoint jumps. Local predicted body gaps, not whole-route ownership, govern concurrent intersection movements.
- Stable vehicle colors, deterministic replay, quiet/busy demand scaling, and 10 Hz screenshot sequences. New crossing paint is visual infrastructure, **not pedestrian behavior**.
- Physical regressions cover red queues, body containment, terminal clearance, local gap acceptance, queued demand, short/infeasible layouts and the original middle-of-intersection pole defect.

## Historical parent-run verification — prior capture

`make test && make check && make build` exited 0 in Herdr terminal `t4z`:

```text
42 simulation tests passed
4 map/object loader tests passed
4 integration tests passed
33 frontend tests passed
15 Python tests passed
```

**98 tests total.** Quiet and busy captures each ran 600 frames through 87–97 simulated seconds, exited 0, and exported exactly 100 frames at 10 Hz. Single-frame images show 88.5 seconds:

- `build/visual/authenticity/iteration-2/quiet.png`
- `build/visual/authenticity/iteration-2/busy.png`
- `build/visual/authenticity/iteration-2/quiet-10hz.mp4`
- `build/visual/authenticity/iteration-2/busy-10hz.mp4`

The independent clip-only A/B review passed the pavement depth-fighting correction. Iteration2 review passes visible crossing placement, busy/quiet contrast and sampled quiet-turn plausibility, but **rejects overall authenticity**: busy release is still serialized, fleet/rider patterns remain too regular, and the normal off-peak reference is unverified. See `build/visual/authenticity/iteration-2/review.md`.

## Known limits

- The previous independent busy capture showed only one vehicle crossing at a time. The local-gap model needs a new independent footage review. Finite braking can exceed the 4 m/s curve target; local constant-speed prediction can still misjudge dense conflicts and blocked exits.
- Imported HCMC still uses synthetic demand and has unsupported short-link, U-turn and body-fit cases. Authored routes can also encounter a named unsupported maneuver. The previously held showcase truck now clears in the tested seed, but other vehicle/band combinations can still be unsupported. Counts are not measured traffic rates; no citywide collision/kinematic certification or large-population performance claim.
- Driver speed, headway, aggressiveness, patience and turn-style variation is opt-in and synthetic. Signal-compliance, opposing-lane filtering and violation counters remain unimplemented. See [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md).
- No pedestrian movement, passenger/bus-service simulation, police management, finance, interactive road construction or dynamic policy editing yet.
- A normal quiet/off-peak matched intersection reference remains unverified. Historical noon congestion is documented as congestion, not mislabeled as quiet.

Run `make run-map MAP=showcase`. Use `--map data/hcm.csv` for real geometry, not a digital-twin claim.
