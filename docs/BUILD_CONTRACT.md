# Phase A shared contract

Goal: runnable technical prototype, not completed management game. Read PLAN.md for context. Shared workspace: edit only assigned paths; no commits, no subagents. Report to orchestrator with `hc send t7`.

## Ownership
- simulation agent: src/sim/** only.
- frontend agent: src/app/** only.
- map agent: src/mapdata/**, tools/**, data/**, docs/MAP.md only.
- orchestrator: root build/docs/integration, review/tests.

## Simulation API (Odin package sim, src/sim)
Use these exact public symbols so work can proceed in parallel:
```
Vec2 :: struct { x, y: f32 }
Node :: struct { position: Vec2 }
Road :: struct { from, to: int, width: f32, speed_limit: f32 }
Vehicle_Kind :: enum { Motorcycle, Car, Bus, Bicycle, Truck, Tricycle }
Vehicle :: struct { road: int, distance, lateral, speed: f32, kind: Vehicle_Kind }
World :: struct { nodes: [dynamic]Node, roads: [dynamic]Road, vehicles: [dynamic]Vehicle, time: f64, completed: int, spawned: int }
init :: proc(w: ^World, nodes: []Node, roads: []Road, seed: u64 = 1)
init_demo :: proc(w: ^World, seed: u64 = 1)
step :: proc(w: ^World, dt: f32)
destroy :: proc(w: ^World)
```
Additional World fields/helpers allowed. init copies network into owned storage. World initialized zero before first init; destroy before reinitialization. Nodes use local meters. Roads directed; vehicle distance from source; lateral measured from centerline, left positive viewed along travel. Vehicle positions derived from road endpoints. Road width is usable carriageway width for that directed edge. Need visible traffic spawn, following/queues and lateral-width-aware occupancy. No requirement for sophisticated routing in spike; label simplifications. Stable seeded behavior; runnable tests via `odin test src/sim`.

## Map API (Odin package mapdata, src/mapdata)
Import ../sim. `load :: proc(path: string) -> (nodes: [dynamic]sim.Node, roads: [dynamic]sim.Road, ok: bool)`; caller deletes returned arrays after sim.init copies. On failure return empty arrays and false, with no leaked resources. Format/import tool owned by map agent. Default path `data/hcm.csv`. Real geometry requires fetched OSM provenance/attribution; do not invent geometry and label real. Aim small Bến Thành-area network with surrounding buffer. If external fetching blocked, document blocker and ship runnable importer/tests without fictional map.

## Frontend
`odin build src/app -out:build/traffic-game`. Import ../sim and ../mapdata and vendor:raylib. Synthetic demo default; explicit key to load real map, label mode and distinguish synthetic demand. 3D pan/orbit/zoom, instanced or basic stylized traffic, road meshes/lines, pause and speeds, HUD counters/help, safe cleanup. Prefer small implementation. Real map use mapdata.load then sim.init; invalid loads keep current scene and show error. No new external GUI dependency. No management claims until implemented.

## Validation
Every owner runs its own available compile/tests; report exact command/result and known limits. Render agent can typecheck before other packages land. Never overwrite sibling work. Ask orchestrator before changing the API. Keep deterministic test fixtures permanently; no debug artifacts. No full-city scale or calibrated realism claims.
