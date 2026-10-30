# XML operational objects: implementation contract

Owners: sim agent src/sim/**; data agent tools/build_map.py tools/test_build_map.py src/mapdata/** maps/** docs/MAP_BUILDER.md; frontend agent src/app/**; orchestrator integration/tests/root docs. No subagents or commits. Shared workspace; preserve sibling files. Report to t7.

## Public simulation API additions

```
Signal_Plan :: struct { node: int, green, amber, clearance, offset: f32 }
Spawn_Point :: struct { road, destination: int, interval: f32, kind: Vehicle_Kind }
Signal_State :: enum { Uncontrolled, Red, Amber, Green }
configure :: proc(w: ^World, signals: []Signal_Plan, spawns: []Spawn_Point) -> bool
signal_state :: proc(w: ^World, road: int) -> Signal_State
```
World exposes owned `[dynamic]Signal_Plan signals`, `[dynamic]Spawn_Point spawns`, `requested:int`, `pending:int`. Other fields internal as needed; Vehicle may gain fields, existing API names remain.

configure called after init on empty simulation. Validate entire configuration (including routes), copy inputs and only change state on success. Failure leaves world usable/unchanged. destroy frees all added allocations. No actual signals if none specified. If spawns nonempty, replace random-anywhere spawning with scheduled typed arrivals; blocked/cap-limited demand remains pending, not discarded. requested = spawned + pending for configured arrivals; spawned = active + completed always. With no spawns, current seeded synthetic demo demand remains intentional test mode.

Each signal controls ALL incoming directed roads at its node, in stable road-index order, one approach green at a time. Each slot = green + amber + clearance; all other approaches red. Offset shifts schedule forward. Supported green/amber/clearance and spawn interval range is 0.05..86400 seconds (20Hz minimum); offset 0..86400, all finite. At least2 incoming approaches and no duplicate controllers for same node. Query state at current world.time. Stops respect front of vehicle (center distance + half vehicle length) and a small stop-line margin. Amber conservatively stops vehicles not yet past stop line; vehicles already committed clear instead of teleporting backward. Include braking/queue release tests. This is exclusive-approach control with instantaneous link transitions, NOT a complete intersection trajectory/conflict model; document ceiling.

Spawn route starts on specified directed road and ends at destination node. Validate route exists through edges wide enough for that vehicle kind. Destination cannot equal start road.from; first road mandatory. Finite precomputed routes are fine; static graph no rerouting needed yet. Count completion at destination, not arbitrary map exit. Default synthetic random mode remains explicit for maps without demand definitions. Maintain120 active-agent cap, retain pending demand and deterministic behavior.

## XML v1 extensions

Existing flat children nodes/roads/buildings retained. Road IDs are required ONLY if referenced by a spawn. `road` with direction=twoway ID resolves to declared forward edge; no ambiguous reverse-road convention.

```
<signal id="junction-light" node="center" green="8" amber="2" clearance="1" offset="0"/>
<spawn id="commuters" road="west-in" destination="east" interval="1.2" kind="motorcycle"/>
```
Allowed spawn kind strings: motorcycle, car, bus, bicycle, truck, tricycle (match enum order, not display names). Strict attrs; default offset=0 permitted, other timings required. Validation matches simulation: unknown references, unreachable/width-incompatible route, duplicate node signal, invalid timings all fail before writing.

Compiler ALWAYS emits stem-objects.csv, including empty header. Third output staged alongside existing road/building files; no multi-file transaction claim. Sidecar format:
```
TRAFFIC_OBJECTS,1
S,node_index,green,amber,clearance,offset
P,road_index,destination_node_index,interval,kind_int
```
Mapdata API:
```
load_objects :: proc(path: string) -> (signals: [dynamic]sim.Signal_Plan, spawns: [dynamic]sim.Spawn_Point, ok: bool)
```
Caller deletes returned flat arrays after configure copies. Loader checks structure/numeric bounds; configure handles topology and semantic validation. Missing optional sidecar on OSM/older authored geometry = no objects. Present invalid sidecar MUST fail map load, not silently degrade. XML now rejects unsupported entities explicitly.

## Frontend integration

Load road geometry + optional objects into temporary World; configure before replacing live World. Existing scene retained on any failure. Detect missing vs present invalid sidecar with core:os. Reset must preserve copied signal/spawn definitions and restart time/RNG; B clears all. Render visible poles/colored heads/stop bars at each signal-controlled approach using the same signal_state used by sim; no per-frame all-pairs road indexing if avoidable. HUD labels scheduled vs random synthetic demand, signal/spawn/pending counts. Existing building sidecar remains separate. Support manual smoke --map path; do not mutate production OSM map to invent signals.

## Examples and verification

Update intersection.xml to a working signal+scheduled demand example (at least two incoming approaches and safe outgoing destination roads). Add signalized.xml if useful; keep existing three scenario paths working. Map builder tests malformed definitions/no-output-damage, runtime tests red front stopping and green release, phase boundaries/clearance/offset, unreachable routes, deterministic scheduled routes and pending conservation. Frontend tests transactional load/reset and geometry. Orchestrator will run end-to-end all maps and screenshots.

## Junction-entry placement correction

Road endpoints are junction centers, not stop lines. `sim.road_offset`, `sim.junction_mouth_depth` and `sim.stop_line_distance` are shared by simulation and rendering. The stop-line helper derives an upstream front limit from connected road widths/offsets, the actual pavement-mouth hull and angled crossing strips, with a 1 m clearance margin. Controllers with no positive usable approach are rejected by runtime configuration. Red/amber admission also refuses a new vehicle whose body would start past the line; existing committed traffic may still clear. This replaces the incorrect endpoint-minus-0.2/0.3 m calculation. Tests must verify placement against actual pavement, not simply repeat the helper's formula.
