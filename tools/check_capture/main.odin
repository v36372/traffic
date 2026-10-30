// Replay the authored 11-source capture benchmark without rendering.
// Run from repository root: odin run tools/check_capture > build/capture-trace.csv
// Frame numbers match --warmup 87 --capture-sequence (600 frames, 10 Hz exports).
package main

import "core:fmt"
import sim "../../src/sim"
import mapdata "../../src/mapdata"

main :: proc() {
    fmt.println("scenario,frame,time,id,kind,road,exit,x,y,forward_x,forward_y,progress,length")
    for scale in ([]f32{0.25, 2}) {
        nodes, roads, ok := mapdata.load("build/maps/showcase.csv")
        assert(ok, "run make maps first")
        w: sim.World
        sim.init(&w, nodes[:], roads[:], 1)
        delete(nodes); delete(roads)
        signals, sources, loaded := mapdata.load_objects("build/maps/showcase-objects.csv")
        assert(loaded && len(sources) == 11)
        for &source in sources { source.interval /= scale }
        assert(sim.configure(&w, signals[:], sources[:]))
        delete(signals); delete(sources)
        for _ in 0..<1740 { sim.step(&w, 0.05) }
        for step in 1..=200 {
            sim.step(&w, 0.05)
            assert(w.spawned == w.completed+len(w.vehicles))
            assert(w.requested == w.spawned+w.pending)
            if step%2 != 0 { continue }
            for v in w.vehicles {
                if !v.motion.active { continue }
                node := w.nodes[v.motion.node].position
                if node.x != -45 || node.y != 0 { continue }
                p, forward := sim.vehicle_pose(&w, v)
                fmt.printf("%s,%d,%.2f,%d,%v,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                    scale < 1 ? "quiet" : "busy", step*3, w.time, v.id, v.kind,
                    v.road, v.motion.exit, p.x, p.y, forward.x, forward.y,
                    v.motion.progress, v.motion.length)
            }
        }
        sim.destroy(&w)
    }
}
