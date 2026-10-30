// Run from the repository root:
// odin run src/sim/check_maps -out:/tmp/traffic-motion-maps
package main

import "core:fmt"
import "core:os"
import sim ".."
import mapdata "../../mapdata"

main :: proc() {
    failed := false
    for name in ([]string{"showcase", "signalized", "hcm"}) {
        path := fmt.aprintf("build/maps/%s.csv", name)
        if name == "hcm" { delete(path); path = fmt.aprintf("data/hcm.csv") }
        nodes, roads, loaded := mapdata.load(path)
        delete(path)
        assert(loaded, "compile fixture maps first with make maps")
        w: sim.World
        sim.init(&w, nodes[:], roads[:], 1)
        delete(nodes)
        delete(roads)
        if name != "hcm" {
            sidecar := fmt.aprintf("build/maps/%s-objects.csv", name)
            signals, sources, drivers, ok := mapdata.load_objects(sidecar)
            delete(sidecar)
            assert(ok && sim.configure(&w, signals[:], sources[:]) && sim.configure_drivers(&w, drivers))
            delete(signals)
            delete(sources)
        }
        collisions, max_node_owners := 0, 0
        owners := make([]int, len(w.nodes))
        for _ in 0..<2400 {
            sim.step(&w, 0.05)
            for &count in owners { count = 0 }
            for v in w.vehicles {
                if v.motion.active {
                    owners[v.motion.node] += 1
                    max_node_owners = max(max_node_owners, owners[v.motion.node])
                }
            }
            assert(w.spawned == w.completed+len(w.vehicles))
            if len(w.spawns) > 0 { assert(w.requested == w.spawned+w.pending) }
            for a, i in w.vehicles {
                for b in w.vehicles[i+1:] {
                    if sim.bodies_overlap(&w, a, b, 0) { collisions += 1 }
                }
            }
        }
        reasons: [7]int
        for issue in w.motion_issues { reasons[int(issue.problem)] += 1 }
        fmt.printf("%s 120s seed=1 mode=%s sources=%d spawned=%d completed=%d active=%d pending=%d issues=%d [degenerate=%d short_approach=%d short_exit=%d u_turn=%d tight_turn=%d terminal_clearance=%d] body_overlap_samples=%d max_node_owners=%d\n",
            name, len(w.spawns) > 0 ? "configured" : "synthetic", len(w.spawns), w.spawned, w.completed, len(w.vehicles), w.pending, len(w.motion_issues), reasons[1], reasons[2], reasons[3], reasons[4], reasons[5], reasons[6], collisions, max_node_owners)
        report_held(&w, name)
        delete(owners)
        failed = failed || collisions > 0
        sim.destroy(&w)
    }
    if failed { os.exit(1) }
}

// Distinguish historical deduplicated issues from actual no-fit vehicles and
// off-network source demand now. Valid-but-occupied alternatives are NOT issues.
report_held :: proc(w: ^sim.World, name: string) {
    held, rejected_sources, rejected_pending, entrance_wait, cap_wait := 0, 0, 0, 0, 0
    for v in w.vehicles {
        if v.motion.active || v.distance < sim.entry_distance(w, v.road, v.kind)-0.001 { continue }
        exit := sim.successor(w, v)
        candidates := sim.connector_laterals(w, v, exit)
        found := false
        problem := sim.Motion_Problem.Short_Exit
        for lateral in candidates {
            _, problem = sim.make_connector(w, v, exit, lateral)
            if problem == .None { found = true; break }
        }
        if !found {
            held += 1
            fmt.printf("  %s held_no_fit id=%d %d->%d kind=%v incoming=%g preferred=%g receiving_trials=%v reason=%v\n",
                name, v.id, v.road, exit, v.kind, v.lateral, v.preferred_lateral, candidates[:], problem)
        }
        delete(candidates)
    }
    for s, i in w.spawns {
        runtime := w.spawn_runtime[i]
        if runtime.pending == 0 { continue }
        desired := sim.source_lateral(w.roads[s.road].width, s.kind, runtime.admitted)
        candidate := sim.Vehicle{road = s.road, kind = s.kind, spawn_index = i, preferred_lateral = desired}
        problem := sim.source_problem(w, candidate)
        actual := desired
        if problem == .None {
            if len(w.vehicles) >= 120 { cap_wait += runtime.pending; continue }
            fits: bool
            actual, fits = sim.fitting_lateral(w, s.road, s.kind, desired)
            if !fits { entrance_wait += runtime.pending; continue }
            candidate.lateral = actual
            problem = sim.source_geometry_problem(w, candidate)
        }
        if problem != .None {
            rejected_sources += 1
            rejected_pending += runtime.pending
            fmt.printf("  %s source_no_fit index=%d pending=%d %d->%d kind=%v incoming=%g reason=%v\n",
                name, i, runtime.pending, s.road, sim.successor(w, candidate), s.kind, actual, problem)
        }
    }
    fmt.printf("  %s actual_held_no_fit=%d source_no_fit=%d source_no_fit_pending=%d entrance_wait_pending=%d cap_wait_pending=%d\n",
        name, held, rejected_sources, rejected_pending, entrance_wait, cap_wait)
}
