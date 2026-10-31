# Continuous motion, straight batching and authored crossings

Run from the repository root:

```sh
make test && make check
odin run src/sim/check_maps -out:/tmp/traffic-motion-maps
```

## Authoritative APIs

`vehicle_pose(w, v) -> (position, forward: Vec2)` owns link and connector poses in east/north coordinates. Forward is unit length. During motion, `road/distance/lateral` still identify the incoming link; renderers must use the authoritative pose, not rebuild a path or reapply offsets. The app interpolates consecutive tick positions and shortest-arc headings by stable trip ID for frame-rate presentation, one tick behind simulation; this does not feed back into physics. `Vehicle.motion.active/node/exit/progress/length` expose the current steering maneuver, not exclusive road ownership. `exit == -1` is a terminal trip. `Vehicle.id: u64` is `spawned+1` at successful admission and resets with the world; no art RNG touches physics.

`crossing_band(w, road) -> (near, far, center_lateral, width: f32, ok: bool)` returns a 4m longitudinal band spanning the **full directed plus reciprocal carriageway union**, including unequal widths. Distances follow the incoming source-to-node axis. `center_lateral` is relative to the **unshifted axis**, so do not add `road_offset` again. A 0.45m lateral envelope covers the app's 0.4m curb landing plus clearance. The far edge clears the full-span junction hull/incident strip overlap by 1m, including skew approaches. The stop bar is another 1m upstream of the near edge. Short/overlapping authored layouts return `ok=false`; there is no fallback paint.

Crossing geometry is operational **only on controlled approaches rendered by the app**. The band query can describe hypothetical geometry, but `stop_line_distance` retains the exact legacy setback for uncontrolled roads/OSM shape edges. `stop_line_for_signals` and `crossing_band_for_signals` validate the explicit proposed signal plan before `configure` changes the live plan/caches. Failed configure leaves the previous configuration intact.

`turn_start_distance` describes the mouth, independently of the upstream STOP line. `junction_clearance_distance` describes rear-clear outgoing geometry, independently of where steering ends. `junction_polygon(w, node, extra=0)` is the single simulation/render convex-hull authority: caller owns/deletes its east/north CCW points; total strip-width addition is 0 for asphalt, 0.4 curb, 4.4 sidewalk. The app's original fixed hull areas and independent cached-triangle body oracle remain unchanged.

## Executed motion and local gaps

Aligned streams share a world-space following controller across links and connectors. Personal acceleration is a ceiling; closing speed and error from a synthetic 0.5m + speed × headway gap determine actual acceleration, bounded by the existing 4m/s² braking rate. Hard movement limits remain emergency checks, not the normal following strategy. Local forecasts use this tick's proposed speed with no 1m/s floor, so slow queues can keep rolling.

- A steering path begins at the stop bar **before** the crossing. On controlled approaches a straight lead carries the center through the entire band before steering begins. Uncontrolled left turns prefer a pavement-fitted curve starting at the entry line, so drivers approach diagonally and can wait partway across for a local gap; if that curve does not fit, the bounded fitter tries later paths, including slightly wider handles that stay in the box rather than snapping 90° at the mouth. After a left-turner has yielded in the junction, the curve target rises from 4 m/s to 6.5 m/s with a 3.5 m/s² floor so the remaining turn is committed rather than a crawl. Other turns can also start at the entry as a fitting fallback. Two-wheelers (motorcycle, bicycle, tricycle) may reverse heading with a left-bulging cubic that stays on incident asphalt; cars, buses and trucks remain an explicit `U_Turn` hold. The lead, 64-segment arc-length cubic and optional straight tail share one metre budget and matching boundary tangents. The tail can finish steering before a narrow receiving strip, with continuous motion until the rear clears the incident hull/strips. Terminal trips also begin at the bar, then retain the existing center-at-end removal convention.
- Nearby vehicles are forecast from observed speeds over a synthetic 1.5-second horizon at 0.05-second samples, shortened toward about 0.95 s for assertive motorcycles; stopped riders are not assigned imaginary forward creep. Earlier entrants get tie priority at predicted conflicts; every actual body blocks regardless of priority. No node, receiving strip, remaining path or outgoing ray is reserved. Multiple turns/crossing streams can proceed together when their local time/space gaps permit. Forecasts stop at the next uncommitted junction, and leaders can turn there without waiting for followers. Road-local following ignores active connectors' frozen incoming metadata; their actual bodies are checked instead. Committed vehicles continue clearing after signal changes; uncommitted red/amber fronts stay behind the shared bar.
- All internal/controlled terminal paths and nonterminal candidates are fitted against the actual incident directed rectangles and shared junction hull, never widened pavement or reduced vehicle dimensions. Bounded local entry/mouth/handle/end variants are tried. Eight body corners/edge centers are checked at <=0.05m. Independent app tests still require construction and actual-triangle containment for all 18 car/motorcycle/bus left/right width fixtures. Failed turn search means `Tight_Turn` **for this fitter**, not proof of physical impossibility.
- Band choice uses the first geometrically feasible guide and waits for its local gap; a temporary obstruction does not trigger a cut across neighboring riders' bands. Static geometry failures still try other bands. This remains fitted-path motion, not adaptive junction weaving or queue filtering.
- World-space oriented body checks span all road IDs. Runtime movement samples at <=0.10m; admission uses local predicted bodies, not a whole-path occupancy scan. These are conservative gameplay samples, not continuous swept-volume certification. Map replay fails on sampled overlap. Bicycle/motorcycle sources cycle width-aware preferred bands after successful admissions; that is packing. With varied driver behavior enabled, vehicles can also make a continuous same-link passing maneuver (12 m, or a 5–6 m motorcycle squeeze when impatient) with fitted body clearance. Motorcycle turn-bias reorders receiving laterals; assertive riders may try the next free band if the preferred one is only occupied. Synthetic-demand successors can take a non-primary non-reversing exit from a keyed wander sample. Opposing-lane filtering and red entry remain unimplemented.

Source validation first checks structural geometry, even if red/occupied entrance fitting cannot supply a body yet. Full body-fit validation uses the **actual fitted incoming slot** and the same `connector_laterals` receiving choices as runtime. A failed center-exit trial cannot reject a valid edge exit. Feasible-but-occupied choices do not emit `Tight_Turn`. Rejected scheduled demand remains pending, counters are conserved, and all choices are deterministic without additional RNG draws.

Static fits are cached per world by road/exit, kind, normalized actual start, incoming/outgoing lateral and receiving-route continuation. Configure/reset/geometry rebuild clears the cache. Occupancy is never cached. This avoids minutes of repeatedly fitting identical unsupported pending trials. Cache size grows with distinct trials until reconfigure/reset; a large-map preprocessing/index can replace it if that ceiling matters. Node/road geometry is static during a run: rebuild the world after geometry edits. Speed-limit edits do not need a rebuild.

## Diagnostics and ceilings

`World.motion_issues` deduplicates `{road, exit, problem}`. Stderr reports observed unsupported attempts with class/incoming/preferred/receiving bands; the app displays the array. Reasons include degenerate links, short approach/exit, near-U-turn (forward dot < -0.95) for classes without a fitted reverse path, `Tight_Turn`, and `Terminal_Clearance`. These are historical observations, not certification of an entire road pair for every vehicle/band. `check_maps` separately reports actual held vehicle IDs/classes/incoming and receiving bands, source no-fit demand, entrance waiting and cap waiting; cached failed alternatives are not counted as held demand.

The compliant baseline remains deterministic. Seeded speed, headway, acceleration, aggressiveness and link-only passing preferences are synthetic; no infractions or opposing-lane filtering were added. The 120-agent cap remains. Local constant-speed prediction is synthetic and can misjudge acceleration, blocked exits or dense cyclic conflicts; congestion can still stop traffic. The old leader-waits-for-followers and outgoing-ray ownership gates are removed. Arbitrary-coordinate floating point behavior is not interval-certified; body and forecast checks remain sampled. There is no congestion-aware routing, unlimited-demand fairness or general citywide deadlock guarantee. Nearby bodies are checked across road/node IDs.

Terminal policy keeps center-at-end removal; it never extends a bus into empty ground to clear paint. Controlled terminals must have their rear > crossing far + 0.5m at that endpoint. ALL internal/controlled terminal bodies must also pass the same actual-asphalt fitting check; failure is `Terminal_Clearance`, retained source pending or a diagnosed held vehicle. An internal destination is not an external exit. Only an explicitly **uncontrolled single-arm endpoint** retains the pre-existing boundary front-overhang abstraction (`terminal_is_external`). A receiving controlled stop/entry remains a boundary even if it is the vehicle's destination: the 21m bus exit-release9m/next-stop7m case is `Short_Exit` for both terminal and continuing routes. Tests reject 3m/6m narrow T-terminal buses and a 3m T-terminal car; positive 6m-bus/3m-car controls include actual forward pavement and require all body points plus last-sample rear clearance. No ghost reservation, off-pavement extension or renderer exemption is used.

Known fitter ceiling: 6m -> 2m straight narrowing, 4.5m car at incoming lateral 2 leaves only 0.05m curb clearance; this center-tangent cubic family currently holds/diagnoses it. The same roads at lateral 1.5 execute a fitted merge plus straight tail. Tests preserve both outcomes without widening pavement. A physical rear-axle trajectory is a future candidate explanation/improvement: body-center trajectory tangent is not a real bicycle-model heading. No wheelbase/calibration claim is made. Other tight two-arm/short OSM layouts remain unproven; accepting a sampled body-fit path does not establish real steering limits or pedestrian safety. On right turns, a body rear can overhang the far crossing edge while the center is already past it and steering remains active; there is no pedestrian behavior here.

The original contradictory 3m-link/4.5m-car recovery test uses valid 40m/80m links now. The bus short-source regression uses an 11m approach where an authored band fits but the 10m bus cannot queue; terminal and continuing routes must reject identically with pending retained. Separate invalid-short controlled layout tests still reject configure.

## Reproducible snapshots

All historical runs below: seed 1, 2400 x 0.05s. Counts satisfy spawned = completed + active, and configured requested = spawned + pending. HCM uses synthetic demand; its current behavior-only sidecar was added after these snapshots.

Historical **10-source** showcase, before this crossing/body-fit slice:

| Stage | Showcase completed | Signalized completed | Max concurrent owners (showcase / signalized) |
| --- | ---: | ---: | ---: |
| One vehicle/node | 14 | 13 | 1 / 1 |
| Straight batching | 23 | 31 | 3 / 5 |

Both historical batching snapshots used identical demand, cap and RNG. HCM at the straight-batch stage: 60 spawned, 9 completed, 51 active, 70 observed issues, zero sampled overlaps.

Historical crossing/body-fit slice, **11-source** showcase (parent added `moto-west-turn`), not a direct demand-controlled comparison to the historical table:

| Map | Sources | Spawned | Completed | Active | Pending | Historical issues | Actual held no-fit | Sampled overlaps |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| showcase | 11 | 137 | 17 | 120 | 8 | 2 | 1 | 0 |
| signalized | 2 | 75 | 29 | 46 | 25 | 0 | 0 | 0 |
| hcm | synthetic | 38 | 6 | 32 | 0 | 87 | 17 | 0 |

In that historical showcase snapshot, bus16->28 incoming0 / receiving0,±0.5 and truck19->0 incoming0 / receiving0,±0.55 produced `Tight_Turn` diagnostics. At 120s truck ID26, road19->0 was held no-fit. No source was geometry-rejected; all 8 pending waited for the cap. Signalized pending25 waited at occupied entrances; none were no-fit. Max simultaneous owners were 3/5/1. No terminal-clearance issue occurred in these three 120s snapshots. A subsequent uncontrolled-entry steering fallback lets ID26 clear in the tested seed; it does not establish support for every vehicle or receiving band.

HCM issues: 9 short approaches, 47 short exits, 9 near-U-turns and 22 bounded-fit failures. Its 17 actual held vehicles at the snapshot are short-exit/U-turn cases; the additional sampled-fit diagnostics are not caused by unauthored zebra setbacks. HCM remains explicitly unsupported for many short shape edges. Zero sampled overlaps is not universal safety or calibrated HCMC throughput.

## Captured focused discharge limit — historical baseline

Parent verification: 98 tests/build and quiet/busy 87–97s captures completed. The parent-owned `tools/check_capture` trace is `build/visual/authenticity/iteration-2/trace.csv`. Its focused-node evidence is **not abreast discharge**:

- Busy: ID7 road6->8 at 88.00–95.20s (25.46731m connector), then ID12 on the same movement at 95.30–97.00s. Maximum simultaneous focused owners is **1**. ID7's sampled world Y moves -5.07500 -> -4.21967 before completion (parent-reported endpoint target -4.125). This is a non-collinear lateral convergence, so the strict straight-only batching exception does not apply.
- Quiet: ID6 road6->25 turns at 88.00–96.10s (28.89461m connector); focused concurrency is also **1**. Fixture and whole-map maximum-owner counts of 3/5 remain valid, but do not demonstrate abreast release in this captured window.
- In that baseline every active connector shared a 4 m/s target ceiling, including straight leads/tails. Roughly seven seconds for the busy connector was a measured gameplay/authenticity limitation, not a claim that the batching predicate was generally broken.

Since that capture, aligned links try a feasible world-space corridor first, and analytically straight connectors use road/class speed targets instead of the universal 4 m/s target. Curved connectors now use local gaps rather than serialization, retaining a 4 m/s target, not a hard limit; finite braking can overshoot it. A replay regression checks the same showcase window and sampled body clearance, but independent footage review and calibration remain outstanding. Opt-in synthetic speed/headway traits are available; filtering is not implemented.
