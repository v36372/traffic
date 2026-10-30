# HCMC Traffic

**Remaining work:** [prioritized roadmap](docs/ROADMAP.md) · [verified status](docs/STATUS.md) · [planned driver behavior](docs/DRIVER_BEHAVIOR.md). This is a prototype; the overall HCMC authenticity target has not been met.

An Odin + raylib traffic-management game in development. **Current milestone: Phase A technical prototype**, not a complete management game or calibrated traffic model.

## Run

Currently targets Linux with Odin's **raylib 6.0 bindings**, Python 3, a C compiler, CMake, curl, tar, sha256sum, and X11/OpenGL development libraries. A working desktop display is required to run the viewer. Run commands from this repository's root:

```sh
make build
./build/traffic-game
```

The first `make build` downloads checksum-pinned **raylib 6.0** and builds it statically under `build/`; subsequent builds reuse it. This avoids the verified local mismatch between Odin's 6.0 bindings and the system's 6.2 library. No system libraries are changed. `tools/bootstrap_raylib.sh` records the source pin and creates the local Odin-root overlay; unsupported binding versions fail clearly. First setup needs network access; the bundled map does not.

`make test` runs simulation, map-loader, integration, frontend geometry, and offline importer checks. `make smoke` builds and renders a short automatic smoke test (requires a display). `make check` type-checks the application. Output binaries live under `build/`.

The default scene is **Lantern Quarter**, an authored six-block neighborhood with 123 buildings and scheduled synthetic traffic. It starts after a 60-second simulation warmup, focused on a busy junction; Home shows the full neighborhood. B opens the small diagnostic junction. See [showcase assumptions](docs/SHOWCASE.md).

`--map data/hcm.csv` loads the real Bến Thành-area OpenStreetMap extract: 389 nodes and 538 directed road segments, roughly a 550 m square before boundary-spanning ways. This is a technical sample, not the proposed full district. **No demand is calibrated to real traffic.** See [map provenance and importer instructions](docs/MAP.md).

Use `./build/traffic-game --help` for controls without opening a window. Start directly on the map with:

```sh
./build/traffic-game --map data/hcm.csv
```

WASD/arrows or right drag pan; middle drag orbits; wheel zooms. Space pauses, +/- changes speed, R resets, M loads the map, B returns to the synthetic scene, Home fits the view, and N faces north. Esc quits.

## Visual review

`make capture` renders a reproducible map screenshot to `build/visual/current.png`. Run `python3 tools/visual_reference.py` to pin an official Cities: Skylines reference, then ask the agent to execute `tools/visual_judge.js` through Pi codemode. The report records image hashes, model, confidence, criterion scores and the next visual priority. See [the repeatable review workflow](docs/VISUAL_REVIEW.md).

## Author test maps

Edit the XML files under `maps/` to define street nodes, directed/two-way roads, and building footprints/types/heights in local meters. Build and run a named scenario:

```sh
make run-map MAP=intersection
make run-map MAP=bottleneck
make run-map MAP=downtown
```

`make maps` compiles these sources to `build/maps/`. Authored XML and imported HCMC OSM both produce the same runtime CSV format: no special simulation path for HCMC. This is a command-line map builder, not a visual editor. See [the map-authoring guide](docs/MAP_BUILDER.md).

## Traffic signals and scheduled trips

XML now supports `<signal>` junction controllers and `<spawn>` typed arrival sources with destination nodes. Signals cycle one approach at a time through green, amber and all-red clearance; vehicles stop before the line and resume on green. Configured trips use finite width-compatible routes. Demand blocked by queues or the 120-vehicle cap remains pending rather than disappearing. Try `make run-map MAP=signalized`.

Maps without authored sources retain the explicit random synthetic-demand mode. Maps without signal definitions remain uncontrolled; the HCMC extract has no invented signal plans. Reset preserves configured objects and restarts their clocks. Present-but-invalid object files reject the map load instead of silently removing controls.

## Buildings

Map mode includes 123 OSM-derived building footprints with simple shaded, solid-color forms: low-rise blocks, apartment slabs, tower/mixed-use buildings and civic landmarks. Tagged heights and floor counts preserve the local skyline, including the 193 m Saigon Centre; missing heights are estimates. No facade textures or windows. Unsupported courtyard/multipolygon buildings and construction/roof-only outlines are omitted rather than filled incorrectly. See [data assumptions](docs/MAP.md).

## Reference-driven traffic checks

Actual HCMC reference provenance is in [HCM_REFERENCES.md](docs/HCM_REFERENCES.md); the live acceptance gate and candid review results are in [AUTHENTICITY.md](docs/AUTHENTICITY.md).

```sh
mkdir -p build/capture-busy
./build/traffic-game --map build/maps/showcase.csv --demand-scale 2 --warmup 87 --capture-sequence build/capture-busy
```

This captures 100 frames over 10 simulated seconds, spanning a green release. Use `--demand-scale 0.25` and a different directory for the quieter synthetic comparison. These multipliers are not measured HCMC demand. `odin run tools/check_capture` replays the same benchmark and emits focused-node vehicle poses as CSV; it does not manipulate the rendered scene.

Crosswalk paint now exists, but pedestrian behavior does not. Seeded driver personalities and deliberate infractions are [planned](docs/DRIVER_BEHAVIOR.md), not implemented. See [current verified status](docs/STATUS.md).

## Current boundaries

- Small-population mixed-width traffic spike, capped at 120 active vehicles; not the plan's proposed 5,000-agent target.
- Cars, motorcycles, bicycles, buses, trucks and cargo tricycles have different dimensions/speeds. Bus passengers, pedestrian networks and observed travel demand are not implemented yet.
- Configured sources have finite routes; random-demand maps can still loop. Vehicles now follow simulation-owned lead/curve/tail paths with reservations and sampled body-pavement checks. Only disjoint straight corridors can cross together; turns/lateral convergence remain conservatively serialized, and unsupported movements are diagnosed rather than teleported. This is not citywide collision or kinematic certification.
- No treasury, police, corruption/trust, save games, or campaign objectives yet.
- Imported road widths/speeds may be inferred; map geometry is not evidence of current regulations or traffic conditions.

## Project documents

- [Research and development plan](docs/PLAN.md)
- [Phase A interface and ownership contract](docs/BUILD_CONTRACT.md)
- [Map pipeline and source limitations](docs/MAP.md)

OpenStreetMap data © OpenStreetMap contributors, licensed under ODbL: https://www.openstreetmap.org/copyright. Map-data licensing is separate from game-code licensing; no game-code license has been selected yet.
