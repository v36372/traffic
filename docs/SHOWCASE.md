# Lantern Quarter showcase

```sh
python3 tools/build_map.py maps/showcase.xml --output build/maps/showcase.csv
./build/traffic-game --map build/maps/showcase.csv
```

Lantern Quarter is an author-designed test neighborhood inspired by the city's urban context, not a map or reconstruction of a real HCMC location. Its 270 × 240 m street grid defines six blocks: narrow shopfront rows address the streets, apartments frame open interiors, and three taller offices punctuate the skyline. Four modest workshops and pavilions occupy parts of the courts while leaving space around them. A broad market hall occupies the southern middle block. The XML is the editable source; the CSV and two sidecars are generated artifacts. See [MAP_BUILDER.md](MAP_BUILDER.md) for the v1 format.

- 12 nodes, 17 two-way physical streets (34 directed edges), 7 m carriageways; 123 buildings: 112 low-rise, 7 apartments, 3 offices and 1 market landmark. The 108 shopfronts are 6 m wide with 0.5 m gaps within each group; four low-rise service buildings are set inside blocks. Heights range from 9 to 72 m.
- The minimum street-centerline-to-footprint clearance is 9 m (5.5 m beyond the carriageway edge); buildings do not overlap. Open block interiors are intentional, not unrendered buildings.
- Three staggered controllers use the simulator's exclusive-approach phases. Eleven scheduled sources have reachable destinations, with motorcycle arrivals predominant; there is no synthetic random demand on this map. This is illustrative demand, not a traffic forecast.
- Motorcycle riders have persistent seeded speed factors of 0.65–1.0, following headways of 0.5–1.5 seconds, acceleration preferences of 1.4–2.6 m/s², and a persona-banded aggressiveness that controls passing pressure, patience, turn-line choice and (on synthetic-demand maps) exit choice. Impatient riders can take a shorter same-carriageway squeeze toward the front of a queue when the body still fits before the stop bar. These settings are synthetic, not measurements of HCMC drivers; opposing-lane filtering and deliberate signal violations are not implemented.
- A green light permits entry when the local gap is clear. Riders can turn and cross together, yielding at predicted nearby conflicts rather than reserving an entire route or receiving road. The synthetic 1.5-second prediction and sampled body checks are not a general deadlock guarantee.
- Startup and smoke capture use the same 60-second warmup. Counts depend on the simulation configuration, not real-world observations.

The initial gameplay view focuses the west signalized junction and surrounding neighborhood. Zoom and orbit to inspect individual streets; Home fits the complete grid. This presentation remains a prototype rather than a visually accepted city reconstruction.
