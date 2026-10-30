# Editable maps

Compile a sample, then load it in the game:

```sh
python3 tools/build_map.py maps/intersection.xml --output build/maps/intersection.csv
./build/traffic-game --map build/maps/intersection.csv
```

`make maps` compiles the samples; `make run-map MAP=bottleneck` builds and runs one. Build the application first as described in the main README. XML is an authoring format, not a runtime dependency: the builder emits `HCM_MAP,1` roads, `intersection-buildings.csv` and `intersection-objects.csv`, even when sidecars are empty. The production OSM importer (`tools/import_osm.py`) keeps its existing geometry artifacts; see [MAP.md](MAP.md). An absent objects sidecar means no authored objects; a present invalid one is an error.

The `HCM_MAP` and `HCM_BUILDINGS` names identify the current file format, not a geographic restriction. The runtime accepts any valid network and building geometry; no HCMC-specific coordinates or topology are required.

## XML v1

```xml
<map version="1" name="Example">
  <node id="west" x="-50" y="0"/>
  <node id="junction" x="0" y="0"/>
  <node id="east" x="50" y="0"/>
  <road id="main" from="west" to="junction" width="3.5" speed="8" direction="twoway"/>
  <road from="junction" to="east" width="3.5" speed="8" direction="twoway"/>
  <signal id="light" node="junction" green="8" amber="2" clearance="1" offset="0"/>
  <spawn id="commuters" road="main" destination="east" interval="2" kind="motorcycle"/>
  <building id="shop" kind="lowrise" height="9">
    <vertex x="10" y="10"/><vertex x="30" y="10"/>
    <vertex x="30" y="25"/><vertex x="10" y="25"/>
  </building>
</map>
```

Coordinates are metres, x east and y north. Node IDs are author-facing strings, mapped to zero-based CSV indexes in node order. Roads reference nodes by ID; crossings connect only when they share a node. A road is a straight segment; `oneway` goes from `from` to `to`, while `twoway` emits both directed edges. Width is metres **per direction**, speed is metres/second. Road IDs are optional; node and building IDs are required. IDs must be nonempty and unique within their element type.

Building kind is `lowrise`, `apartment`, `office`, or `landmark`; height is metres. Supply 3–256 vertices without repeating the first vertex. Outlines must be simple and nondegenerate; either winding is accepted and emitted counterclockwise. Holes are not supported. The builder rejects unknown attributes and elements, invalid references and numeric values. To match the runtime loaders, coordinates must be strictly between -10,000,000 and 10,000,000, width and speed strictly between 0 and 100, and height strictly between 0 and 1000; values are validated as runtime 32-bit floats. Roads must remain nonzero length at that precision. At least one road is required.

Signals and spawns have required IDs, unique within their type. A signal requires a node with at least two incoming directed roads and no other signal on that node. Green, amber and clearance durations and spawn interval are seconds in [0.05, 86400]; offset is seconds in [0, 86400], default 0. All are finite f32 values. Each signal cycles incoming roads in road-index order: one green, then amber, then all-red clearance, with all other approaches red. Offset shifts the cycle forward. This is exclusive-approach control with instantaneous road transitions, not a full intersection trajectory or conflict model.

Spawns require a road ID, destination node ID, interval and one of `motorcycle`, `car`, `bus`, `bicycle`, `truck`, `tricycle`. The specified forward directed edge is mandatory, including for a `twoway` road. The destination cannot be its source node and must be reachable using roads wide enough for that vehicle. Spawned demand follows scheduled typed trips; maps without spawns use synthetic random demand.

Object sidecars start `TRAFFIC_OBJECTS,1` and contain `S,node_index,green,amber,clearance,offset` and `P,road_index,destination_node_index,interval,kind_int` records. Vehicle kind integers follow the order above, starting at zero. The `mapdata.load_objects` caller owns both flat arrays and deletes them after `sim.configure` copies them. The loader validates structure and numeric bounds; configuration validates topology.

The builder validates before writing and stages all three files before replacing any. A validation failure leaves existing outputs untouched. Each replacement is atomic, but the three are not transactional if the operating system fails during replacement. Input and output paths must be distinct. Other objects, including bus stops, are future catalog ideas, not implemented; unsupported XML elements are rejected.

## Object catalog

| Group | Constructible now | Next, not yet accepted by XML |
|---|---|---|
| Streets | Nodes, one-way/two-way segments, per-direction width and speed | Lanes/turn permissions, bridge levels, scheduled vehicle restrictions |
| Urban context | Low-rise, apartment, tower/mixed-use and landmark footprints/heights | Terrain, vegetation and water |
| Junction control | Exclusive-approach signal controllers with amber and all-red intervals | Turn-specific phases, yield/stop priority, pedestrian phases |
| Travel demand | Typed spawn sources, arrival intervals, width-compatible destinations/routes | Pedestrian paths/crossings, bus stops/routes/passengers, time-of-day demand |
| Management | None yet | Police stations/posts/shifts, construction projects and financial scenario settings |

An object is added to the supported schema when its runtime behavior and validation exist. Unknown objects fail compilation; this avoids test maps that look configured but silently do nothing.

Runtime signal configuration also validates usable junction-entry space. A road's destination node is the junction center, so stop lines must be upstream of the adjoining pavement rather than immediately before the node. A controller on an approach too short to provide that space rejects the map load; positive road length alone is insufficient. Compiler validation does not replace this runtime geometry check.
