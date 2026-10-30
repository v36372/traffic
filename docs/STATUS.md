# Prototype status — 2026-10-30

**Not yet an authentic/calibrated HCMC traffic simulation.** Live acceptance evidence and remaining work: [AUTHENTICITY.md](AUTHENTICITY.md).

## Current changes — macOS arm64

- The local raylib bootstrap builds on macOS arm64. `make test && make check && make build` passed with 142 tests; a macOS smoke run opened and closed the viewer successfully. CI configuration covers Linux x86_64 and macOS arm64 but has not yet run on GitHub.
- Aligned junction links now try to preserve the incoming world-space band. In the 87–97 s busy showcase replay with synthetic motorcycle preferences and passing enabled, the focused junction had up to two simultaneous connector owners and 53 of 100 sampled frames had at least two; this is not a visual authenticity review or a continuous collision proof. Straight connectors use road/class speed targets; curved movements retain a 4 m/s target and generally exclusive reservation.
- A reconstructed default-demand west-cross scenario verifies that a straight rider can enter during green while a previous turning rider's remaining path is conservatively clear. The current overtaking-enabled full replay has a different queue, and turning riders at its front still wait through the full 88–96 s green phase. Broader green-phase delays remain unresolved.
- At an uncontrolled showcase bend, a turning body may now begin steering from its reserved entry if the existing pavement fit passes. The previously held truck clears in the seed-1 120 s replay, with no sampled body overlaps; other unsupported bends remain possible.
- The showcase and HCMC extract enable persistent synthetic motorcycle speed/headway/acceleration preferences, seeded personas and link-only passing when a maneuver fits. A reusable driver factory and parameterized state machine are documented in [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md). OSM import reports directed reachability and uses supported numeric speed tags; see [MAP.md](MAP.md).
- Quiet and busy captures from before motorcycle preferences were enabled exported 100 frames each to `build/visual/latest-quiet/` and `build/visual/latest-busy/`; clips are `build/visual/latest-quiet.mp4` and `build/visual/latest-busy.mp4`. They remain unreviewed and do not show the current driver configuration.

## Working

- Odin/raylib 3D viewer with orbit/zoom, a populated authored neighborhood, and imported HCMC roads plus 123 OSM-derived buildings. Simple low-rise, apartment, tower and landmark forms; explicit source heights preserved.
- XML map compiler for streets/buildings, signal controllers, and typed scheduled origins/destinations. Five authored examples; the showcase now has 11 sources including an explicit visible left-turn route.
- Shared junction geometry drives pavement, crossing paint, stop bars, pole placement and vehicle constraints. Continuous simulation-owned lead/curve/tail motion replaces endpoint jumps. Straight disjoint corridors can share a node, and a straight rider can enter when a turning rider's remaining path is conservatively clear; other movements remain conservatively serialized.
- Stable vehicle colors, deterministic replay, quiet/busy demand scaling, and 10 Hz screenshot sequences. New crossing paint is visual infrastructure, **not pedestrian behavior**.
- Physical regressions cover red queues, body containment, terminal clearance, reservations, queued demand, short/infeasible layouts and the original middle-of-intersection pole defect.

## Historical parent-run verification — prior capture

`make test && make check && make build` exited 0 in Herdr terminal `t4z`:

```text
42 simulation tests passed
4 map/object loader tests passed
4 integration tests passed
33 frontend tests passed
15 Python tests passed
```

**98 tests total.** Quiet and busy captures each ran 600 frames through 87–97 simulated seconds, exited 0, and exported exactly 100 frames at 10 Hz. Single-frame images show 88.5 seconds:

- `build/visual/authenticity/iteration-2/quiet.png`
- `build/visual/authenticity/iteration-2/busy.png`
- `build/visual/authenticity/iteration-2/quiet-10hz.mp4`
- `build/visual/authenticity/iteration-2/busy-10hz.mp4`

The independent clip-only A/B review passed the pavement depth-fighting correction. Iteration2 review passes visible crossing placement, busy/quiet contrast and sampled quiet-turn plausibility, but **rejects overall authenticity**: busy release is still serialized, fleet/rider patterns remain too regular, and the normal off-peak reference is unverified. See `build/visual/authenticity/iteration-2/review.md`.

## Known limits

- The previous independent busy capture showed only one vehicle crossing at a time. Aligned bands and straight-connector speed targets improve the replay's sampled concurrency, but new footage has not yet received independent review. Curved movements remain serialized; finite braking can exceed the 4 m/s curve target.
- Imported HCMC still uses synthetic demand and has unsupported short-link, U-turn and body-fit cases. Authored routes can also encounter a named unsupported maneuver. The previously held showcase truck now clears in the tested seed, but other vehicle/band combinations can still be unsupported. Counts are not measured traffic rates; no citywide collision/kinematic certification or large-population performance claim.
- Driver speed and following-headway variation is opt-in and synthetic; signal-compliance, filtering and patience variation remain proposed. No deliberate violation counters or filtering behavior. See [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md).
- No pedestrian movement, passenger/bus-service simulation, police management, finance, interactive road construction or dynamic policy editing yet.
- A normal quiet/off-peak matched intersection reference remains unverified. Historical noon congestion is documented as congestion, not mislabeled as quiet.

Run `make run-map MAP=showcase`. Use `--map data/hcm.csv` for real geometry, not a digital-twin claim.
