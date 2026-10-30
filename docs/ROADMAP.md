# Remaining work

The current deliverable is a tested Odin/raylib prototype, **not a finished game or a calibrated HCMC digital twin**. The latest independent review still requests changes. See [STATUS.md](STATUS.md) for verified functionality, [PLAN.md](PLAN.md) for the original research, and [AUTHENTICITY.md](AUTHENTICITY.md) for the investigation record.

## 1. Fix authentic moving traffic — highest priority

- **Busy discharge needs visual review and further work.** The prior recorded 87–97 s showcase window serialized the focused junction. The current headless replay with driver preferences and passing enabled produces up to two simultaneous owners; it has not yet passed independent footage review. Straight connectors now use road/class speed targets while curved connectors retain a conservative 4 m/s target, not a hard limit. Investigate safe compatible converging/turning paths, admission ordering and physically calibrated curve speeds. Do not raise the population cap or manipulate screenshots to disguise low throughput.
- **Improve path-fitting coverage.** The previously held showcase truck clears its uncontrolled bend in the tested replay, but other incoming bands and imported OSM short links, U-turns and tight curves remain unsupported. Explore movement-aware routing and multi-edge body occupancy/shape-node treatment. The bounded center-tangent cubic fitter is not proof that a real vehicle cannot make a turn. Investigate rear-axle/body-reference kinematics before adding increasingly arbitrary curve trials.
- **Preserve physical invariants.** Keep bodies on unchanged pavement, maintain stop/crosswalk geometry, prevent accidental red entry, retain pending demand and hold reservations through clearance. Internal terminal destinations are not automatically external exits. Keep the strict 18-case render-pavement oracle and independent terminal tests.
- **Verify actual footage.** Repeat quiet/busy runs using identical geometry, camera and seed. Include a visible turn and a complete green release. Judge recordings/frame sequences as well as stills; aggregate concurrency counters alone are insufficient.

**Exit criterion:** representative busy traffic demonstrably releases in plausible groups, turns remain physically contained, replay/accounting tests pass, and independent review accepts the moving scene with stated limitations.

## 2. Implement persistent driver personalities

Opt-in synthetic speed and following-headway distributions are implemented in [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md). Reaction, amber decisions, filtering, patience and infractions remain unimplemented.

- Add reaction/amber decisions on top of the seeded desired-speed and following-headway distributions. Keep traits persistent and independent of appearance and arrival randomness; retain a fully compliant deterministic baseline.
- Implement signal compliance, filtering tendency and bounded patience only when their decision and movement models exist. Decide at meaningful events, not with a random coin flip every frame.
- Add continuous lateral filtering and merge/gap acceptance before permitting opposing-lane encroachment or other prohibited maneuvers. Source lateral slots are not lane changing.
- Expose only implemented controls in XML/UI. Add separate attempted, completed and blocked infraction counters. Clearing a light already entered legally must not count as a new violation; crashes need explicit mechanics rather than numerical overlap.

**Exit criterion:** changing behavior settings produces reproducible, visible behavioral differences without disabled physical guards or unimplemented knobs.

## 3. Improve reference evidence and real-map fidelity

- Obtain an identified normal quieter/off-peak intersection reference, preferably paired with busy conditions at a comparable location/view. Current references include historical noon **congestion**, not a verified quiet matched pair. Keep source time/context uncertainty visible; never substitute holiday or lockdown emptiness.
- Acquire permitted motion footage suitable for measuring queue release, speeds, gaps and vehicle proportions. Treat current demand scales and parameter values as synthetic until calibrated; public-camera failures and absent EXIF are documented in [HCM_REFERENCES.md](HCM_REFERENCES.md).
- Extend OSM processing beyond default widths and basic numeric speed tags: lane/access/turn information, conditional speeds, bridge/tunnel levels and unsupported building multipolygons/courtyards. The importer reports weak/strong road components and can audit directed reachability for specified OSM node pairs; neither audit checks vehicle fit, turn restrictions or near-miss junctions. Import signals only with evidence; do not invent observed timing plans.
- Replace the real-map random first-successor demand model with defensible origins/destinations and time-of-day demand. Resolve short geometry chains without losing actual intersections or hiding unsupported routes.

**Exit criterion:** map assumptions and traffic calibration have traceable provenance; unsupported geometry and unknown timings are explicit, not silently replaced.

## 4. Complete traffic-management gameplay and population

- Add actual pedestrian networks, crossing occupancy and signal phases; zebra paint currently has no pedestrian behavior. Extend bicycles and motorcycle filtering with safe lateral motion.
- Add bus stops, routes, passengers and service frequency; differentiate truck weight/size/access classes and model cargo tricycles appropriately. Current vehicle dimensions alone do not constitute transit or freight simulation.
- Implement editable lanes/turn permissions, one-way/two-way changes, vehicle bans and hourly restrictions, more flexible signal phases and junction priority. Add a visual editor/save workflow on top of the existing XML constructors.
- Implement road widening/upgrades, grade-separated bridges, construction disruption and costs; then public funding/budgets and the traffic-police staffing, salary, enforcement and corruption/unrest systems described in the original plan.

**Exit criterion:** players can detect a bottleneck, make a funded intervention, and observe a causally meaningful result rather than only watch a demonstration.

## 5. Production readiness

- Profile before raising the 120-agent cap. Current scans, static movement-fit caching and conservative reservations have explicit scaling limits; establish budgets for larger populations and long-running sessions.
- Add save/load and scenario versioning, reliable replay diagnostics, deployment packaging/asset paths and cross-platform build validation. A pinned GitHub Actions workflow covers Linux x86_64 and macOS arm64 but has not yet been run on GitHub. The local raylib 6.0 bootstrap supports Linux and macOS arm64; other platforms remain unverified.
- Improve interaction, accessibility and visual readability using real references. Trees/frontage context may help, but do not substitute decoration for correct traffic. Photorealistic facades are not required.
- Confirm distribution licensing/attribution for data and dependencies. Reference news/stock photos are analysis material, not game assets; do not bundle them without appropriate rights.

## Reproduce the current baseline

```sh
make test && make check && make build
make run-map MAP=showcase
```

For matched captures, use separate output directories:

```sh
mkdir -p build/quiet build/busy
./build/traffic-game --map build/maps/showcase.csv --demand-scale 0.25 --warmup 87 --capture-sequence build/quiet
./build/traffic-game --map build/maps/showcase.csv --demand-scale 2 --warmup 87 --capture-sequence build/busy
odin run tools/check_capture > build/capture-trace.csv
```

Each capture exports 100 frames over 10 simulated seconds. `build/` is intentionally ignored: a fresh clone contains source, tests, authored maps and OSM data, **not local recordings, downloaded reference photos or build binaries**. Recreate those artifacts locally; public provenance and review conclusions are retained in the documentation.
