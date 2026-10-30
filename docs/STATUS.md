# Prototype status — 2026-10-30

**Not yet an authentic/calibrated HCMC traffic simulation.** Live acceptance evidence and remaining work: [AUTHENTICITY.md](AUTHENTICITY.md).

## Working

- Odin/raylib 3D viewer with orbit/zoom, a populated authored neighborhood, and imported HCMC roads plus 123 OSM-derived buildings. Simple low-rise, apartment, tower and landmark forms; explicit source heights preserved.
- XML map compiler for streets/buildings, signal controllers, and typed scheduled origins/destinations. Five authored examples; the showcase now has 11 sources including an explicit visible left-turn route.
- Shared junction geometry drives pavement, crossing paint, stop bars, pole placement and vehicle constraints. Continuous simulation-owned lead/curve/tail motion replaces endpoint jumps. Straight disjoint corridors can share a node; other movements remain conservatively serialized.
- Stable vehicle colors, deterministic replay, quiet/busy demand scaling, and 10 Hz screenshot sequences. New crossing paint is visual infrastructure, **not pedestrian behavior**.
- Physical regressions cover red queues, body containment, terminal clearance, reservations, queued demand, short/infeasible layouts and the original middle-of-intersection pole defect.

## Latest parent-run verification

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

- In the actual focused busy capture, only one vehicle crosses at a time. A slight lateral transition triggers conservative serialization; every active connector is capped at 4 m/s. Passing parallel-discharge fixtures does not prove authentic discharge in this scene.
- Imported HCMC still uses synthetic demand and has unsupported short-link, U-turn and body-fit cases. Authored routes can also encounter a named unsupported maneuver. Counts are not measured traffic rates; no citywide collision/kinematic certification or large-population performance claim.
- Driver speed/gap/compliance/filtering/patience variation is **proposed, not implemented**. No deliberate violation counters or filtering behavior. See [DRIVER_BEHAVIOR.md](DRIVER_BEHAVIOR.md).
- No pedestrian movement, passenger/bus-service simulation, police management, finance, interactive road construction or dynamic policy editing yet.
- A normal quiet/off-peak matched intersection reference remains unverified. Historical noon congestion is documented as congestion, not mislabeled as quiet.

Run `make run-map MAP=showcase`. Use `--map data/hcm.csv` for real geometry, not a digital-twin claim.
