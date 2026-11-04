# HCMC Traffic

**Remaining work:** [prioritized roadmap](docs/ROADMAP.md) · [verified status](docs/STATUS.md) · [driver behavior](docs/DRIVER_BEHAVIOR.md). This is a prototype; the overall HCMC authenticity target has not been met.

An Odin + raylib traffic-management game in development. **Current milestone: Phase A technical prototype**, not a complete management game or calibrated traffic model.

## Run

Targets Linux x86_64/aarch64 and macOS arm64 with Odin's **raylib 6.0 bindings**, Python 3, a C compiler, CMake, curl, and tar. Linux also requires sha256sum and X11/OpenGL development libraries; macOS uses shasum and the Xcode command-line tools. A working desktop display is required to run the viewer. Run commands from this repository's root:

```sh
make build
./build/traffic-game
```

The first `make build` downloads checksum-pinned **raylib 6.0** and builds it statically under `build/`; subsequent builds reuse it. This avoids the verified local mismatch between Odin's 6.0 bindings and the system's 6.2 library. No system libraries are changed. `tools/bootstrap_raylib.sh` records the source pin and creates the local Odin-root overlay; unsupported binding versions fail clearly. First setup needs network access; the bundled map does not.

`make test` runs simulation, map-loader, integration, frontend geometry, and offline importer checks. `make smoke` builds and renders a short automatic smoke test (requires a display). `make check` type-checks the application. Output binaries live under `build/`.

The default scene is a **signalized four-way** with mixed vehicle types and left, through, right and U-turn movements, including some U-turns before the box. Pole-mounted 3D heads show red, amber and green; authored `countdown` adds the LED timer module. It starts after a 30-second warmup, looking at the crossing; Home shows the full map. `make run-map MAP=showcase` loads **Lantern Quarter**, the six-block neighborhood (60-second warmup, west-crossing camera). B opens the small diagnostic junction. See [showcase assumptions](docs/SHOWCASE.md).

`--map data/hcm.csv` loads the real Bến Thành-area OpenStreetMap extract: 389 nodes and 538 directed road segments, roughly a 550 m square before boundary-spanning ways. This is a technical sample, not the proposed full district. **No demand is calibrated to real traffic.** See [map provenance and importer instructions](docs/MAP.md).

Use `./build/traffic-game --help` for controls without opening a window. Start directly on the map with:

```sh
./build/traffic-game --map data/hcm.csv
```

WASD/arrows or right drag pan; middle drag orbits; wheel zooms. Space pauses, +/- changes speed, R resets, M loads the map, B returns to the synthetic scene, Home fits the view, and N faces north. Esc quits.

For uncapped gameplay (higher GPU/CPU usage), run `./build/traffic-game --uncapped`. This removes the game's 60 FPS limiter, not desktop/GPU synchronization; traffic simulation speed stays unchanged. The default remains capped at 60 FPS. Smoke/sequence captures retain their fixed 60 FPS clock.

## Simulation debugger (CLI-first)

Run `make -s debug` for a persistent native session. Use `seek 250` and `list stopped` for a fresh run, or `load test-cases/default-250-driver98.cbor` for the preserved failure; then `inspect`, `probe`, `step 1`, `save`, `check`, `quit`. Commands return one JSON object each, including the actual following leader/gap/acceleration, physical clearance and independent movement probe. No display, mouse automation or OCR is involved. `make replay-case CASE=test-cases/default-250-driver98.cbor` runs the preserved behavior assertion. `make editor` remains an optional visual inspector. See [the debugger workflow](docs/SIMULATION_EDITOR.md).

## Measure frame pacing

The top-right HUD shows FPS and the last frame time. For a repeatable stationary-camera measurement of the normal real-clock loop:

```sh
make build
./build/traffic-game --benchmark 15
./build/traffic-game --benchmark 15 --uncapped
```

Each run excludes 2 seconds of rendering warmup, measures 15 seconds, prints mean FPS, frame-time p50/p95/p99/max, slow-frame counts (>25/>50 ms), simulation/draw-submission/end-present timings, and exits. Controls are disabled except Esc (aborts with a nonzero exit). Duration accepts 1..300 seconds. Keep the window visible and avoid competing workloads; compare the same map, demand, warmup, window size and build flags. `--uncapped` removes only the game's 60 FPS limiter, not desktop/GPU synchronization. Screenshots/smoke capture are rejected because their fixed clock and disk I/O are not gameplay performance.

60 FPS is a 16.67 ms frame budget; tail frame times reveal hitches hidden by an average. End/present includes batch flush, swap, limiter wait and input polling, **not isolated GPU time**. Physics runs at **20 Hz**, but vehicle positions and shortest-arc headings are **interpolated every rendered frame**. Presentation trails simulation by one tick (50 ms at 1×), without predicting through queues or changing physics. Pause freezes the displayed phase; reset/map changes clear history. These measurements do not certify compositor/display presentation timing.

Local measurements (Radeon 780M, 1280×800, 4× MSAA, powersave CPU governor; live desktop, not a controlled hardware benchmark):

| 15-second run | Original FPS / p99 ms | Optimized FPS / p99 ms |
| --- | ---: | ---: |
| Default demand | 54.94 / 38.19 | 60.00 / 16.71 |
| 2× demand, 120 active | 36.55 / 69.32 | 57.87 / 24.77 |

`make build` now uses `-o:speed` instead of Odin's default minimal optimization. Collision queries reuse candidate poses/forecasts within each read-only call and avoid repeating identical stopped/stopped predictions; no forecast samples, collision margins or safety gates were removed. An instrumented pre-change simulation replay attributed about 59% of simulation time to `local_gap_clear` and 27% to `body_clear` at 2× demand (not percentages of total frame time). A frozen-baseline comparison matched every vehicle field after every tick for 1,540 ticks at each demand level. The committed `collision_queries_test.odin` compares optimized queries to uncached predictions, alongside existing collision/containment tests. Raw local evidence: `build/performance-final-{capped,busy}.log`, `build/performance-function-profile.log`, `build/performance-identical.log`. These timings precede presentation interpolation; dense traffic still hitches.

## Visual review

`make capture` renders a reproducible map screenshot to `build/visual/current.png`. Run `python3 tools/visual_reference.py` to pin an official Cities: Skylines reference, then ask the agent to execute `tools/visual_judge.js` through Pi codemode. The report records image hashes, model, confidence, criterion scores and the next visual priority. See [the repeatable review workflow](docs/VISUAL_REVIEW.md).

## Author test maps

Edit the XML files under `maps/` to define street nodes, directed/two-way roads, and building footprints/types/heights in local meters. Build and run a named scenario:

```sh
make run-map MAP=intersection
make run-map MAP=showcase
make run-map MAP=bottleneck
make run-map MAP=downtown
```

`make maps` compiles these sources to `build/maps/`. Authored XML and imported HCMC OSM both produce the same runtime CSV format: no special simulation path for HCMC. This is a command-line map builder, not a visual editor. See [the map-authoring guide](docs/MAP_BUILDER.md).

## Traffic signals and scheduled trips

XML now supports `<signal>` junction controllers and `<spawn>` typed arrival sources with destination nodes. Build a plan with `sim.make_signal` / `default_signal_spec`: green, amber and all-red clearance seconds, cycle offset, and `countdown` for remaining-time digits. Signals cycle one approach at a time; vehicles stop before the line and resume on green. Configured trips use finite width-compatible routes. Demand blocked by queues or the 120-vehicle cap remains pending rather than disappearing. Try `make run-map MAP=signalized`.

Maps without authored sources retain the explicit random synthetic-demand mode. Maps without signal definitions remain uncontrolled; the HCMC extract has no invented signal plans. Reset preserves configured objects and restarts their clocks. Present-but-invalid object files reject the map load instead of silently removing controls.

## Buildings

Map mode includes 123 OSM-derived building footprints with simple shaded, solid-color forms: low-rise blocks, apartment slabs, tower/mixed-use buildings and civic landmarks. Tagged heights and floor counts preserve the local skyline, including the 193 m Saigon Centre; missing heights are estimates. No facade textures or windows. Unsupported courtyard/multipolygon buildings and construction/roof-only outlines are omitted rather than filled incorrectly. See [data assumptions](docs/MAP.md).

## Reference-driven traffic checks

Actual HCMC reference provenance is in [HCM_REFERENCES.md](docs/HCM_REFERENCES.md); the live acceptance gate and candid review results are in [AUTHENTICITY.md](docs/AUTHENTICITY.md).

```sh
mkdir -p build/capture-busy
./build/traffic-game --map build/maps/showcase.csv --demand-scale 2 --warmup 87 --capture-sequence build/capture-busy
```

This captures 100 frames over 10 simulated seconds, spanning a green release. Captures use the same interpolation as gameplay: at each exported frame the displayed vehicles trail the simulation/HUD clock by 50 ms. Use `--demand-scale 0.25` and a different directory for the quieter synthetic comparison. These multipliers are not measured HCMC demand. `odin run tools/check_capture` replays the same benchmark and emits focused-node **simulation-tick** vehicle poses as CSV (not delayed presentation poses); it does not manipulate the rendered scene.

Crosswalk paint now exists, but pedestrian behavior does not. Opt-in seeded speed and headway preferences are available in authored XML; filtering and deliberate infractions are [not implemented](docs/DRIVER_BEHAVIOR.md). See [current verified status](docs/STATUS.md).

## Current boundaries

- Small-population mixed-width traffic spike, capped at 120 active vehicles; not the plan's proposed 5,000-agent target.
- Cars, motorcycles, bicycles, buses, trucks and cargo tricycles have different dimensions/speeds. Bus passengers, pedestrian networks and observed travel demand are not implemented yet.
- Configured sources have finite routes; random-demand maps can still loop. Vehicles follow simulation-owned lead/curve/tail steering paths with sampled body-pavement checks. Local rolling gap acceptance allows concurrent turns/crossing streams without whole-path or receiving-road reservations; unsupported geometry is diagnosed rather than teleported. This is not citywide collision or kinematic certification.
- No treasury, police, corruption/trust, save games, or campaign objectives yet.
- Imported road widths/speeds may be inferred; map geometry is not evidence of current regulations or traffic conditions.

## Project documents

- [Research and development plan](docs/PLAN.md)
- [Phase A interface and ownership contract](docs/BUILD_CONTRACT.md)
- [Map pipeline and source limitations](docs/MAP.md)

OpenStreetMap data © OpenStreetMap contributors, licensed under ODbL: https://www.openstreetmap.org/copyright. Map-data licensing is separate from game-code licensing; no game-code license has been selected yet.
