// make record-case TIME=250 DRIVER=98; make replay-case
package main

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import sim "../../sim"
import mapdata "../../mapdata"
import replay ".."

main :: proc() {
    w: sim.World
    defer sim.destroy(&w)
    args := os.args[1:]
    if len(args) >= 1 && args[0] == "--session" {
        if len(args) > 2 { usage(); return }
        path := "build/maps/intersection.csv"
        if len(args) == 2 { path = args[1] }
        os.exit(run_session(path))
    }
    if len(args) == 5 && args[0] == "--record" {
        seconds, time_ok := strconv.parse_f64(args[2])
        selected, id_ok := strconv.parse_u64(args[3])
        if !time_ok || math.is_nan(seconds) || math.is_inf(seconds) || seconds < 0 || seconds > 3600 || !id_ok || selected == 0 { usage(); return }
        if !load_debug_map(&w, args[1]) { os.exit(2) }
        ticks := int(math.round(seconds/0.05))
        for _ in 0..<ticks { sim.step(&w, 0.05) }
        found := false
        for v in w.vehicles { if v.id == selected { found = true; break } }
        if !found { fmt.eprintf("Driver %d is not present at %.2fs\n", selected, w.time); os.exit(2) }
        inspect(&w, selected)
        if !replay.save(&w, args[4], selected) { fmt.eprintln("Could not save case"); os.exit(2) }
        fmt.printf("Recorded %s: complete native world, selected=%d, assertion=advance >=0.5m or finish within20s\n", args[4], selected)
        return
    }
    if len(args) != 1 { usage(); return }
    selected, duration, minimum, loaded := replay.load_case(&w, args[0])
    if !loaded { fmt.eprintf("Invalid or unreadable replay case: %s\n", args[0]); os.exit(2) }
    inspect(&w, selected)
    result := replay.check(&w, selected, duration, minimum)
    probe, available := replay.steering_probe(&w, selected)
    fmt.printf("Independent steering probe: available=%v distance=%.2fm curvature=%.6f/m (current bodies + asphalt)\n", available, probe.distance, probe.curvature)
    fmt.printf("%s selected=%d duration=%.2fs progress=%.6fm required=%.3fm completed=%v overlaps=%d off_pavement=%d\n",
        "PASS" if result.passed else "FAIL", selected, duration, result.progress, minimum, result.completed, result.overlaps, result.off_pavement)
    if !result.passed { os.exit(1) }
}

load_debug_map :: proc(w: ^sim.World, path: string) -> bool {
    nodes, roads, loaded := mapdata.load(path)
    if !loaded { return false }
    defer delete(nodes); defer delete(roads)
    next: sim.World
    sim.init(&next, nodes[:], roads[:], 1)
    objects := fmt.aprintf("%s-objects.csv", strings.trim_suffix(path, ".csv"))
    defer delete(objects)
    signals, spawns, drivers, ok := mapdata.load_objects(objects)
    defer delete(signals); defer delete(spawns)
    if !ok || !sim.configure(&next, signals[:], spawns[:]) || !sim.configure_drivers(&next, drivers) { sim.destroy(&next); return false }
    sim.destroy(w)
    w^ = next
    return true
}

usage :: proc() {
    fmt.eprintln("Usage: replay-case --session [MAP.csv]\n       replay-case CASE.cbor\n       replay-case --record MAP.csv SECONDS DRIVER_ID CASE.cbor\nNative state only; no window, renderer, OCR, or altered traffic.")
    os.exit(2)
}

inspect :: proc(w: ^sim.World, selected: u64) {
    fmt.printf("time=%.2fs seed=%d vehicles=%d spawned=%d completed=%d pending=%d\n", w.time, w.behavior_seed, len(w.vehicles), w.spawned, w.completed, w.pending)
    for v, i in w.vehicles {
        if v.id != selected { continue }
        p, f := sim.vehicle_pose(w, v)
        exit := sim.successor(w, v)
        if v.motion.active { exit = v.motion.exit }
        accel := sim.following_acceleration(w, v, i, v.driver.traits.headway, v.driver.traits.acceleration)
        fmt.printf("driver=%d kind=%v mode=%v road=%d exit=%d speed=%.6f pose=(%.6f,%.6f) heading=(%.6f,%.6f) path=%.6f/%.6f following_accel=%.6f\n",
            v.id, v.kind, v.driver.mode, v.road, exit, v.speed, p.x,p.y,f.x,f.y,v.motion.progress,v.motion.length,accel)
        a := v.driver.actor
        fmt.printf("  state=%s intent=%v no_progress_seconds=%.6f retry_after=%.6f attempts=%d state_since=%.6f\n", sim.driver_state_path(a.state), a.intent, a.no_progress, v.driver.replan_after, a.attempts, a.since)
        fmt.printf("  execution=%v maneuver=%v\n", a.last, a.plan)
        for other in w.vehicles {
            if other.id == selected { continue }
            q, g := sim.vehicle_pose(w, other)
            if sim.vec_distance(p, q) > 5 { continue }
            fmt.printf("  nearby id=%d road=%d speed=%.6f pose=(%.6f,%.6f) heading=(%.6f,%.6f)\n", other.id, other.road, other.speed,q.x,q.y,g.x,g.y)
        }
    }
}
