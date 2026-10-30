# Phase A Bến Thành map

## Source and attribution

`data/hcm.osm` is the unmodified OpenStreetMap API XML response retrieved **2026-10-30 14:20 UTC** from:

https://api.openstreetmap.org/api/0.6/map?bbox=106.697,10.770,106.702,10.775

The bbox is west,south,east,north in degrees, around Bến Thành; returned ways can extend beyond it. SHA-256:

- `data/hcm.osm`: `72c8bd1849c9446502bb8d70744ca7dc4cb2501bd821601b1119cbe2fb317eef`
- `data/hcm.csv`: `111b4fb21897afba1a7e6ee8884cac814c9cc765a895c35139f0cfd06028f678`
- `data/hcm-buildings.csv`: `5efcffbab7af7e0954e174206696227f419b6e06656cf2e41e7d125881bde9c1`

**© OpenStreetMap contributors.** OpenStreetMap data is available under the Open Database License (ODbL): https://www.openstreetmap.org/copyright and https://opendatacommons.org/licenses/odbl/1-0/. Attribution must remain visible wherever this map is used. The XML and reproducible transformation are included to facilitate access to the underlying and adapted data; review ODbL database/produced-work and redistribution obligations before release. The code has separate licensing from the OSM data.

## Rebuild and load

```sh
python3 tools/import_osm.py data/hcm.osm data/hcm.csv --buildings data/hcm-buildings.csv
sha256sum data/hcm.osm data/hcm.csv data/hcm-buildings.csv
python3 -m unittest discover -s tools -p 'test_*.py'
odin test src/mapdata
```

The stdlib-only importer reads OSM XML, not PBF or Overpass JSON. It writes deterministic UTF-8 CSV: `HCM_MAP,1` header, `N,x,y` local-metre nodes, then `E,from,to,width,speed_limit` directed edges; indexes are zero-based. X is east, Y north, equirectangular projection centered at 10.7725°N, 106.6995°E. The exact Odin API is `mapdata.load(path) -> (nodes, roads, ok)`; caller deletes both arrays after `sim.init` copies them. A failed load returns empty arrays and false.

To audit a particular directed node pair, add repeatable `--route FROM_OSM_NODE TO_OSM_NODE` arguments to the import command. For example, `--route 411917839 411917837 --route 411917837 411917839` reports the first route reachable in four edges and the reverse unreachable in this extract. The audit distinguishes missing nodes from unreachable pairs and reports a shortest edge count, not distance. It checks imported directed topology only: it does not account for vehicle width, turn restrictions, body fit or external roads outside the extract.

## Meaning and limits

Topology and explicit oneway tags are OSM-derived. Each consecutive pair of OSM node IDs becomes an edge; only shared node IDs connect, so crossing ways are not joined based on geometry. `oneway=-1`, explicit two-way, and implied roundabout/motorway one-way are handled. Coordinates are rounded to millimetres; selected ways with missing nodes or zero-length segments after rounding are rejected before writing output. The importer reports weakly and strongly connected road components, listing the five largest of each when there is more than one, with node counts and example OSM node IDs. Only nodes on imported roads are counted. Strong components contain nodes mutually reachable following edge directions; different components can still be reachable in one direction. The extract has 389 nodes, 538 directed edges, one weak component and 107 strong components (the largest has 239 nodes). The clipped boundary and one-way roads contribute to this result; it is not by itself evidence of a topology error. The audit does not identify erroneous connections, near-miss junctions or reachability of particular origin/destination pairs.

Widths are **inferred constants** of 3.5 m per directed edge. The importer converts numeric OSM `maxspeed` tags (km/h by default, or explicit `km/h` or `mph`) to metres per second, rounded to 0.001 m/s; directional `maxspeed:forward` and `maxspeed:backward` override the general tag for their respective OSM way direction. Missing values use an **inferred** 8.33 m/s. Unsupported or nonpositive explicit values reject the import rather than silently using the default. The importer reports edge counts by speed source; this extract has 100 OSM-tagged and 438 default-speed directed edges. Tags are not field measurements, and conditional, vehicle-specific and implicit legal speed rules are not modeled. Reciprocal edges share centerline nodes; rendering may offset them without changing topology. Traffic demand is **synthetic**, not observed or calibrated. Only a subset of motor-traffic highway classes is imported. General `access` and `motor_vehicle` no/private are excluded; more specific, conditional or time-dependent access, turn restrictions, barriers, signals, lane counts, bridge layers and other relation semantics are not modeled. Geometry is real source data, not a verified current legal or operational network. The small API bbox is a technical spike, not the planned 2–4 km² district or an audited routing buffer.

For imported ways the audit counts ignored `layer`, `bridge`, `tunnel`, `turn:lanes` (including directional variants), and any `*:conditional` tags, with up to five source way IDs per category. It also counts ignored `type=restriction` relations referencing at least one imported way and reports up to five relation IDs. These counts indicate tags or relations present, not confirmed errors or modeled restrictions. The checked-in extract reports two ways with `layer`, one with `tunnel`, two with turn lanes, 22 with conditional tags, and 15 restriction relations referencing imported ways; some categories can overlap, and a counted relation may be incomplete at the extract boundary. Other unmodeled tags and relations are not exhaustively listed.

The hand-authored `data/hcm-objects.csv` enables the same synthetic per-motorcycle speed and headway variation as the showcase. It contains no signals or scheduled demand, and is not generated from OSM or calibrated to observed driving; see [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md).

## Building sidecar

The optional `--buildings` output uses the road projection and millimetre rounding, without inventing positions. The renderer derives the sidecar from the map basename: `hcm.csv` → `hcm-buildings.csv`. Custom maps need not have a sidecar; traffic does not depend on it. UTF-8 CSV starts `HCM_BUILDINGS,1`; each record is `B,height,kind,count,x0,y0,...`, with height in metres and no repeated closing vertex. Kind is 0 Lowrise, 1 Apartment, 2 Office (including tall mixed-use towers), or 3 Landmark. These are **inferred rendering categories**, not verified use or architectural detail.

Explicit numeric `height` in metres (also ` m` suffix) takes precedence; otherwise numeric `building:levels` × 3 m, including fractional levels; otherwise a deterministic 9 m. Invalid explicit dimensions are skipped, not replaced by defaults. Heights must be finite, at least 0.000001 m and below 1000 m. Values are not field measurements. Building and amenity tags identify landmarks (markets, civic, religious and libraries); apartments/residential at least 6 m are apartments; office/commercial/retail at least 6 m or any building at least 30 m is the Office/tower category. Everything else is lowrise. Tall generic buildings can be hotels or mixed use, not necessarily offices. Names are retained in OSM but not used to guess classification. Buildings are simple untextured extrusions, not detailed models.

The extract has 129 building-tagged ways. Of these, 3 roofs, 2 with construction tags, and 1 multipolygon member are excluded; 123 ways are exported. An additional building multipolygon relation is unsupported. The importer reports `unsupported_outlines=2` (the excluded tagged way and the separate relation, not all intentionally filtered roofs/construction); a tagged member of a building relation can contribute to both counts. All way members of any multipolygon are excluded, regardless of role or tags, to avoid filling holes. Unclosed, missing-node, degenerate, self-intersecting, repeated-vertex, invalid-height, and over-256-vertex outlines are also counted as unsupported. Only simple closed ways are exported; relation assembly, holes, roofs, parts semantics and overlap removal are not modeled. Separate OSM building parts can overlap their parent footprints. This is not a complete building inventory.

For the checked-in artifact, height sources are 3 explicit, 74 levels-derived, 46 default; inferred kinds are 98 lowrise, 9 apartment, 10 office/tower, 6 landmark. `mapdata.load_buildings(path)` returns `(buildings, ok)`: missing or malformed files return false and no buildings; an empty valid sidecar succeeds. The loader validates finite coordinates, positive finite height, kind, 3–256 vertices, and simple nondegenerate polygons, and normalizes winding CCW in simulation x/y. Call `mapdata.destroy_buildings(buildings)` to release the caller-owned array and all footprints. Mesh owners must validate triangulation separately. Polygon validation is O(n²), bounded to 256 vertices; use a spatial index if that ceiling is raised.
