# HCMC authenticity: acceptance gate and evidence

**Status: target not met.** The latest independent review requests changes. The initial reference-driven work window has ended; no implementation agents or recurring work remain active. Remaining implementation priorities are consolidated in [ROADMAP.md](ROADMAP.md). Seeded driver variation is documented in [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md).

## Goal and evidence standard

Use real HCMC traffic as the reference, including intersection scenes and busy/quieter conditions. Distinguish photographed appearance from behavior that requires footage or measurements. Do not substitute Cities: Skylines imagery for HCMC evidence, label unknown capture hours as off-peak, use holiday/lockdown emptiness as normal traffic, or claim calibrated demand without measurements.

The target is a recognizable, approximately authentic playable prototype, not a digital twin or a universal collision/kinematic certification. Simple solid-color architecture is acceptable.

## Acceptance gate

Independent review must assess both synthetic demand levels against appropriate real references:

1. Credible geometry: poles on sidewalks, stop bars upstream, bodies on carriageways, no obvious overlap or clipping.
2. Recognizable motorcycle-dominated traffic: readable riders, side-by-side waiting, and plausible moving groups rather than a sterile single-file queue.
3. Clear busy/quieter contrast without invented observed traffic counts or source times.
4. HCMC streetscape proportions: narrow frontages, varied heights, sidewalks and markings; photorealistic textures are unnecessary.
5. Motion checked with sequences and tests, not a single still. No turn teleportation or accidental red bypass accepted as realism. Deliberate infractions must be configured, counted and physically simulated, with a compliant baseline retained.

A vision-model verdict is evidence, not a substitute for user judgment. Missing reference or motion evidence prevents an overall pass.

## Reference corpus

[HCM_REFERENCES.md](HCM_REFERENCES.md) records nine downloaded photographs, sources, captions, hashes and rights limitations. The set includes caption-confirmed rush-hour context and historical noon congestion. It does **not** establish a normal quiet/off-peak matched intersection pair. Unknown hours remain unknown.

The noon photograph is congested and carries historical weekend/construction context; it must not be presented as a quiet baseline. Public-camera access failures and missing capture-hour metadata are documented, not bypassed or guessed. Third-party images stay under ignored `build/references/hcm/` as analysis material, not game assets.

## What the iterations established

| Iteration | Evidence and result |
|---|---|
| Initial signal correction | The user identified poles inside the junction. The original code confused a road endpoint at the junction center with the entrance. Shared geometry and independent rendered-pavement tests replaced tests that repeated the same incorrect formula. Same-camera visual review passed this specific placement correction. |
| Baseline comparison | Real-reference review rejected single-file motorcycle strings, uniform rider/vehicle appearance and missing pedestrian markings. The quiet/busy synthetic contrast was visible; that alone was insufficient. |
| Continuous-motion pass | Simulation-owned paths replaced endpoint jumps. Stopped abreast packing and rider variety passed component review, but actual release/turn evidence was insufficient. Review exposed a pavement line fan and repeated large-body colors. |
| Depth A/B | Pinned raylib's near-plane setting provided insufficient depth precision for the road layers. A clip-only same-camera comparison removed the fan without visible loss of markings, curbs, buildings or vehicles. Other views were not granted a blanket visual pass. |
| Crossing/body pass | Shared crossing geometry, straight lead-in, curve and reserved tail were tested against actual asphalt. Independent body tests caught curb-cutting turns; paths were corrected without expanding pavement or shrinking bodies. Terminal exceptions were separately reproduced and corrected. |
| Latest visual review | Crossing placement, absence of pavement fans, busy/quieter contrast and sampled quiet-turn plausibility passed. Fleet/rider variety and busy discharge remained partial. Overall verdict: **FAIL / REQUEST_CHANGES**, not a claim that every physical subsystem failed. |

## Latest verified baseline

Parent-run `make test && make check && make build` exited 0 with **98 tests**: 42 simulation, 4 map/object loader, 4 integration, 33 frontend and 15 Python tests.

The current authored showcase has **11 sources**, including an explicit left-turn route at the focused junction. Only its central arterial was widened to 5.5 m per direction; side streets remain 3.5 m per direction. These are authored dimensions, not surveyed calibration. Independent tests check building/pavement clearance and fixed junction-hull areas so path failures cannot be hidden by silently growing roads.

Matching quiet/busy captures use seed 1, demand scales 0.25 and 2, the same geometry/camera, and 87 s warmup. Each ran 600 render frames to 97 s and exported exactly 100 frames at 10 Hz. Single-frame images show 88.5 s. Encoded videos were verified as 100 frames, 10 fps and 10 seconds.

Local evidence under `build/visual/authenticity/iteration-2/`:

- `quiet.png`, `busy.png`, `quiet-motion/`, `busy-motion/`
- `quiet-10hz.mp4`, `busy-10hz.mp4` and timestamped motion panels
- `verification.log`, `runtime.sha256`, `trace.csv`, `review.md`

These generated files are intentionally not committed. Recreate captures using [ROADMAP.md](ROADMAP.md); `odin run tools/check_capture` replays the same benchmark and emits focused-node poses without modifying the rendered scene.

## Actual flow, not only passing fixtures

The focused replay contradicts an overly broad claim of abreast discharge:

| Scenario | Observed focused movement |
|---|---|
| Quiet | Motorcycle ID 6, road 6→25, turning during 88–96.1 s |
| Busy | Motorcycle ID 7, road 6→8, alone during 88–95.2 s; ID 12 follows from 95.3 s |

Focused maximum concurrent ownership was **one** in both captures. A mild lateral transition is conservatively serialized, and all active connectors have a 4 m/s ceiling. Parallel unit fixtures and aggregate map concurrency do not prove authentic discharge in this actual scene. This is the highest-priority remaining movement discrepancy.

Final 120 s, seed 1 diagnostic runs on the new 11-source benchmark recorded:

| Map | Spawned / completed / active | Pending | Historical issues / currently held no-fit |
|---|---|---|---|
| Showcase, configured | 137 / 17 / 120 | 8 | 2 / 1 |
| Signalized fixture, configured | 75 / 29 / 46 | 25 | 0 / 0 |
| HCMC, synthetic | 38 / 6 / 32 | 0 | 87 / 17 |

All three runs had zero sampled body-overlap detections. That is a bounded sampled result, not proof of arbitrary continuous safety. The HCMC issues include 9 short approaches, 47 short exits, 9 U-turns and 22 tight-turn cases. It has no authored signal/crossing plans. Historical 10-source metrics must not be used as a like-for-like comparison with this 11-source scene.

## Physical rules that must not regress

- One shared geometry authority drives pavement, stop bars, poles, crossing bands and movement constraints. Stop/admission, steering entry, steering completion and rear-clearance release are distinct boundaries.
- Source validation tests the actual admitted lateral slot using the same receiving alternatives as runtime admission. A failed speculative candidate is not a genuinely unsupported movement if another candidate fits.
- Internal or controlled terminal destinations must respect downstream red/entry boundaries, fit the actual pavement and clear their painted band before removal. Infeasible cases retain pending demand or are diagnosed upstream. Only an uncontrolled single-arm endpoint retains the documented external-network overhang abstraction.
- Straight disjoint corridors can share a node; curves and convergence remain conservatively serialized. No renderer-only fake turns or disabled overlap guards.
- The unchanged 18-case body-containment oracle, independent terminal render-pavement tests, fixed hull areas, conservation checks and deterministic replay tests remain required.

Detailed geometry, cache and kinematic limits are in [src/sim/MOTION.md](../src/sim/MOTION.md) and [src/app/RENDERING.md](../src/app/RENDERING.md). The sampled, bounded center-tangent cubic fitter is not proof that a real vehicle cannot perform a rejected maneuver.

## Next work

The prioritized [roadmap](ROADMAP.md) starts with actual moving-group compatibility and straight-through speed policy, then richer lateral filtering. Seeded speed, gap, aggressiveness, patience and same-carriageway filtering are implemented; signal compliance and opposing-lane cuts remain planned. Intentional, observable infractions must not be used to relabel physical bugs as Vietnamese traffic behavior.

The authenticity goal must not be marked complete until the remaining flow and reference-evidence gaps have been addressed and reviewed again.
