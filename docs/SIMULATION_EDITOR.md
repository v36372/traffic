# Simulation debugger and native regression cases

## Primary interface: CLI

`make -s debug` starts one persistent simulation session. Read commands from stdin; receive one JSON object per command on stdout. Diagnostics use stderr. This uses the same world, demand, drivers and 50ms ticks as `make run`, without a display server, GUI clicks, or OCR.

```sh
printf '%s\n' 'load test-cases/default-250-driver98.cbor' 'inspect' 'step 1' 'check 20 0.5' 'quit' | make -s debug
```

Commands:

- `seek SECONDS`, `step [TICKS]`: advance deterministically; earlier seeks restart the same world definitions and seed. A tick is 50ms; seek accepts 0–3600s.
- `select ID`, `inspect [ID]`: inspect exact driver pose/path, the actual following-controller leader/gap/acceleration, physical next-step/gap clearance, available steering probe, and nearby bodies. `inspect ID` also selects it.
- `list [stopped|candidates]`, `probe [METRES]`: enumerate drivers or test an independent forward arc of 0.25–5m (default 0.75m). A short free arc is not proof of a complete passing corridor. For a fresh scene, `seek 250`, then choose a present ID from `list stopped`; historical IDs may already have completed after physics changes.
- `save [PATH]`, `load [PATH]`: save/load the full native case, including selection and assertion settings. Default path is `build/debug-case.cbor`; paths may contain spaces.
- `check [SECONDS [MIN_METRES]]`, `help`, `quit`: check a clone against progress/safety assertions. Check defaults to 20s and 0.5m, or the loaded case settings. A failed check makes the session exit 1; invalid input makes it exit 2. Commands can continue after an error, but the final exit status retains failures.

Inspect and list include the actor state path, intent, no-progress timer, retry deadline, attempts, state-entry time, and execution and maneuver feedback (including blocker and reason); mode remains the legacy projection. The JSON output is for observation; version 2 CBOR snapshots retain bit-exact numeric state, including actor fields. Historical version 1 scenes initialize actors from their preserved physical state on load. `make record-case` / `make replay-case` below are convenient single-shot commands. A frozen case can also be loaded directly with `load test-cases/default-250-driver98.cbor`.

## Find the first persistent stalls

```sh
make -s debug-build
python3 tools/find_stalls.py --until 600 --output build/stall-audit/current
```

This drives the persistent CLI from time zero, sampling every five native ticks (250ms). It tracks exact zero speed **and unchanged position**, rather than one stopped frame. After two seconds of continuously available local steering space it saves the whole scene, inspector data, and 0.75/1.5/3m probe results. `temporal-report.json` also retains transient stops and stops with no detected local space; red queues are not automatically bugs. `--step-ticks 1` samples every50ms. Probes cover ordinary road links as well as committed junctions, so handing off cannot hide a stalled driver. A local arc remains a diagnostic suspect, not a certificate of a complete compliant passing route.

The unchanged first-stop scenes are in `test-cases/captured-first-stop-*.cbor` (drivers109,199,184). Later `captured-link-stop-*.cbor` scenes cover two link stalls, and `captured-lane-stop-t168.00-driver211.cbor` covers a safety-vetoed lane change with3m straight ahead clear. Native regressions retain the surrounding traffic and require useful progress, no body overlap and no sampled pavement violation. The lane case must advance within3s, rather than silently waiting another ten seconds for traffic to rearrange.

## Optional visual interface

Run `make editor`. This is a simulation inspector, not a road/building authoring tool. F3 toggles it in the ordinary game.

## Reproduce a problem

1. Set **Run to time** to `250` and click **Run deterministic seek**. Seeking runs bounded tick batches without waiting for wall-clock time. An earlier target resets the same map, objects and seed first.
2. Click a vehicle, or press Tab to cycle through vehicles. The inspector shows its ID, mode, speed, road/exit, signal, following acceleration and path progress. The yellow line is its actual remaining guide; outlined bodies and projected guide blockers come from simulation state, across road IDs.
3. A green arc means the independent diagnostic found at least 0.75m of forward movement clear of current bodies and asphalt. This bounded constant-curvature probe is not the production planner and does not reserve space or prove a whole route is available.
4. Click **Save .cbor**, then **Check**. Saving records every vehicle and maneuver, signal/demand clocks, RNG, seed, counters and arbitration sequence. Checking clones this exact state and requires the selected trip to move at least 0.5m or finish within 20 simulated seconds, with no body overlap or sampled junction-pavement violation. Surrounding traffic stays present; the editor's world does not advance during the check.
5. **Load .cbor** restores `build/debug-case.cbor`. Single-step advances exactly one 50ms tick and pauses. Space plays/pauses; R resets; Home fits; N faces north; normal pan/orbit/zoom controls remain available outside the sidebar.

Paused editor views show authoritative current poses, not the normal game's one-tick-delayed interpolation. Run-to input is snapped to 50ms and accepts 0–3600 seconds. Case checks use sampled geometry, not a continuous swept-volume guarantee. Signal colors refer to the incoming approach; an already committed junction driver is not required to re-enter on green.

## Run the same assertion without a window

`make record-case TIME=250 DRIVER=ID` captures a present driver from a fresh default-map run at seed 1 into `build/debug-case.cbor`. Obtain `ID` from the current session's list; the ID from a historical frozen case need not still exist in a fresh run. Override `MAP`, `TIME`, `DRIVER`, or `CASE` as needed. `make replay-case` loads the frozen native state, prints the driver's pose and neighbors plus the independent steering probe, and returns a failure exit status when the progress/safety assertion fails. It does not use screenshots, OCR, isolated vehicles, or synthetic replacements for the surrounding traffic.

A preserved reproduction of the reported default-scene bug is included:

```sh
make replay-case CASE=test-cases/default-250-driver98.cbor
```

**Before the fix:** driver 98 moved 0m over the next 20s despite a free forward probe. The tangent-projection following controller falsely treated an angled body's bounding projection as a leader at its standstill gap. The controller now measures contact along the actual guide; receiving bands can change, and a maneuver can continue beyond an obsolete transition endpoint without crossing the next entry/signal line. The same preserved snapshot now exceeds 5m of progress in 20s, without sampled overlap or off-pavement movement.

`test-cases/default-250-near-exit.cbor` preserves the sibling refusal where driver 184 had only 0.089m left on its old guide but a free 1.5m steering arc. It previously moved 0m; the saved replay now passes its progress/safety assertion. Replay tests cover all six originally blocked central drivers, this near-exit case, safe/nonmutating checks, and exact snapshot replay. The snapshots are not regenerated when fixing the behavior: the original surrounding traffic stays intact.

`test-cases/default-250-probe-limit.cbor` guards a diagnostic limitation: its clear 3m arc ends in the opposing strip, heading away from the intended exit. `available=true` means only sampled local body/asphalt clearance, not a complete legal passing route. No arbitrary movement assertion is imposed on that driver merely because the short probe is clear.

To inspect that preserved case in the editor, copy it to `build/debug-case.cbor` and click **Load .cbor**. The selected driver is restored too. Case files use versioned CBOR because the installed standard JSON float formatter loses f32 precision; the snapshot test compares every vehicle field and subsequent ticks after saving/loading. These are development regression snapshots, not a supported player save-game format.

Native `EDITOR SEEK`, `EDITOR SAVE`, `EDITOR LOAD` and `EDITOR CHECK` events expose results to automation without reading screen pixels.

`test-cases/default-250-receiving-queue.cbor` captures a fresh-run driver66 stopped behind an offset car despite a same-direction side corridor. Its regression independently validates a7m forward path with all120 bodies frozen, including full-body pavement and receiving-strip bounds, then requires the real simulation to advance at least5m within20s. The driver may hand off in place before using the same-link steering search; it need not retain the old junction maneuver to pass. Run `make replay-case CASE=test-cases/default-250-receiving-queue.cbor`.
