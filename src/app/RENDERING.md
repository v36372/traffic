# Pavement depth precision

The iteration-1 independent image review reports a pale line fan over the upper-left asphalt. The following isolates depth precision as a hypothesis; mathematical separation alone is not an image acceptance result.

## Verified pinned implementation

- `build/deps/raylib-6.0/src/rlgl.h` defaults `RL_CULL_DISTANCE_NEAR=0.05`, `RL_CULL_DISTANCE_FAR=4000`.
- `src/external/glfw/src/window.c` in that dependency requests a 24-bit depth buffer by default. The actual framebuffer bit depth has not been queried here.
- Pinned `rcore.c::BeginMode3D` builds its perspective using `rlGetCullDistanceNear/Far`; the Odin binding exposes `rlgl.SetClipPlanes`.
- Cached road layers remain at 0, 0.015, 0.03, 0.045 m. No mesh inflation, pavement topology, simulation geometry or camera framing changes.

For perspective depth, `depth(z) = f/(f-n) - fn/((f-n)z)` and one approximate 24-bit unit at distance z is `z²(f-n)/(fn(2²⁴-1))` metres along camera depth. A vertical layer separation projects to `0.015*sin(pitch)`.

At the existing 113.166261 m entry camera / pitch 1.30:

| Clip range | Approximate unit | 15 mm layer separation |
| --- | ---: | ---: |
| 0.05 .. 4000 m | 0.01526647 m | 0.947 depth units |
| 2.263325 .. 1000 m | 0.00033650 m | 42.952 depth units |

`camera_clip_planes` uses near `max(0.1, distance*0.02)`, far `max(1000, distance+max(extent*4,distance*3))`. Far includes the finite ground plane; near scales with orbit distance instead of requiring a district-scale camera to retain a 5 cm plane. Very shallow/horizon-length views still have finite depth precision. `gl.SetClipPlanes` is applied before each `BeginMode3D` so Home/orbit/zoom use the same rule.

## Repeatable check / image evidence

Run `ODIN_ROOT="$PWD/build/odin-root" odin test src/app -define:RAYLIB_SHARED=false`.

`test_gameplay_clip_range_separates_pavement_without_clipping_scene` reproduces default-clip sub-unit separation, checks more than four units across actual entry/Home pavement, verifies actual building vertices survive entry/Home clipping at wide/portrait aspects, and checks close-zoom target precision. Existing independent pavement/pole/building-clearance tests are unchanged.

Clip-only A/B artifacts were built as `build/traffic-game-depth-before` and `build/traffic-game-depth-after` (also copied to `build/traffic-before-clip` / `build/traffic-after-clip`), with identical motion/map state and no vehicle palette difference. These binaries intentionally precede the separate larger-vehicle palette slice.

Independent inline image review in `build/visual/authenticity/depth-ab/review.md` reports **PASS for targeted fan removal** at the unchanged entry camera / demand 2× / 61.5 s. The reviewer sees the fan before, clear asphalt after, and retained markings, curbs, buildings and vehicles without apparent new clipping. This is single-view evidence, not a visual certification of every Home/zoom/orbit position or traffic behavior. Home/close coverage above is mathematical regression coverage; actual image review there remains separate.

## Crossing paint

`crossings.odin` caches zebra paint and curb landings only for configured signal approaches with a valid `sim.crossing_band`. The shared simulation API supplies axis-relative lateral center/full reciprocal span and longitudinal band edges. App rendering adds no carriageway offset and never shifts stop bars independently. Unsupported short/adjacent-envelope layouts omit paint with a terminal diagnostic.

The shared band is 4 m deep, its far edge 1 m upstream of the incident hull/strip envelope, with the signal stop bar 1 m upstream of its near edge. Half-metre white stripes use a 1 m transverse pitch. Each 0.4 m transverse landing lies outside the full carriageway span, on the existing sidewalk/curb band, within the simulation's 0.45 m clearance. These add no asphalt and do not inflate existing road layers; paint uses the existing stop-bar marking height (0.055 m).

The HUD explicitly says crossing paint only, no pedestrian simulation. No pedestrian actors, yielding, crossing safety or real-world signal layout calibration is implied. The simulation owns a separate turn entry after a reserved straight lead-in; physical body containment is checked independently against actual cached asphalt in `containment_test.odin`. A failing containment test is a path defect or unsupported turn layout, never a reason to widen rendered pavement.
The focused containment oracle samples four body corners and four edge centers every at most 0.1 m of actual simulation connector progress. All 18 centered-path cases pass: motorcycle/car/bus, left/right, equal 3.5 m, 5.5/3.5 m arterial/side-street, and unequal reciprocal-width fixtures. No unsupported-case exception was added. Fixed asphalt hull-area checks (104.125 and 181.5 m² for the equal and widened orthogonal fixtures) guard against pavement expansion concealing a path defect. Other configurations/positions can still be diagnosed `Tight_Turn`; this is not global turn certification, and the new crossing appearance still requires independent image review.
The independent terminal fixture also rejects a controlled bus whose rear clears the band but whose front exceeds a perpendicular 6 m strip (front x=5 m, actual asphalt ends at x=3 m). A forward 6 m strip exists only in the positive test fixture; its accepted connector must keep all eight body samples on actual cached asphalt. Sampling covers the owned connector interval, not the separately named external-boundary spawn-overhang abstraction. No game pavement is widened for this check.

## Benchmark lifecycle

Current showcase demand has **11 authored synthetic sources**, including `moto-west-turn`: west arterial `h01` to `n12`, motorcycle interval 8 s. This adds explicit turn coverage at the focused crossing; it is not a measured HCMC count or fleet-share change. App fixture checks require the 11-source sidecar and this source's route through the focused junction.

Historical iteration-0/1 and clip A/B captures/throughput numbers used the earlier **10-source scenario** and remain historical evidence, not directly comparable throughput measurements of the new configuration. Frozen executable files do not freeze their external CSV map/object files. New comparisons must use the same regenerated 11-source files, camera, warmup, demand labels and capture cadence on both sides; new 10 Hz sequences use frames 6..600 every 6 frames, not the historical 2 Hz cadence.
