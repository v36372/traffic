package main

import "core:bufio"
import "core:encoding/json"
import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import sim "../../sim"
import replay ".."

Session :: struct { world: sim.World, selected: u64, duration: f64, minimum: f32 }
Reply :: struct {
    op: string,
    ok: bool,
    error: string,
    time: f64,
    seed, selected: u64,
    active, spawned, completed, pending: int,
}
Driver_View :: struct {
    id: u64,
    kind: sim.Vehicle_Kind,
    mode: sim.Driver_Mode,
    state_path: string,
    intent: sim.Driver_Intent,
    no_progress_seconds: f32,
    retry_after: f64,
    last_execution_feedback, last_maneuver_feedback: sim.Driver_Feedback,
    attempts: u64,
    state_since: f64,
    road, exit: int,
    speed: f32,
    position, forward: sim.Vec2,
    path_progress, path_length: f32,
    following: sim.Following_Observation,
    next_body_clear, next_gap_clear, steering_available: bool,
    steering: replay.Steering_Probe,
}
Neighbor :: struct { id: u64, road: int, speed: f32, position, forward: sim.Vec2 }

encode_reply :: proc(s: ^Session, op: string, ok: bool, details: any = nil, error: string = "") -> (string, bool) {
    w := &s.world
    event := Reply{op, ok, error, w.time, w.behavior_seed, s.selected, len(w.vehicles), w.spawned, w.completed, w.pending}
    data, err := json.marshal(event, {use_enum_names = true})
    if err != nil { return "", false }
    defer delete(data)
    // The installed JSON codec cannot marshal an `any` struct field. Marshal
    // the concrete payload separately; both fragments remain codec-produced.
    payload: []byte
    if details != nil {
        payload, err = json.marshal(details, {use_enum_names = true})
        if err != nil { return "", false }
    }
    defer delete(payload)
    return fmt.aprintf("%s,\"details\":%s}", string(data[:len(data)-1]), string(payload) if details != nil else "null"), true
}

reply :: proc(s: ^Session, op: string, ok: bool, details: any = nil, error: string = "") {
    data, encoded := encode_reply(s, op, ok, details, error)
    assert(encoded, "Cannot encode native debugger response")
    defer delete(data)
    fmt.println(data)
}

find_driver :: proc(s: ^Session, id: u64) -> (sim.Vehicle, int, bool) {
    for v, i in s.world.vehicles { if v.id == id { return v, i, true } }
    return {}, -1, false
}

driver_view :: proc(s: ^Session, v: sim.Vehicle, index: int) -> Driver_View {
    w := &s.world
    view := Driver_View{id = v.id, kind = v.kind, mode = v.driver.mode, road = v.road, speed = v.speed}
    view.state_path = sim.driver_state_path(v.driver.actor.state)
    view.intent = v.driver.actor.intent
    view.no_progress_seconds, view.retry_after = v.driver.actor.no_progress, v.driver.replan_after
    view.last_execution_feedback, view.last_maneuver_feedback = v.driver.actor.last, v.driver.actor.plan
    view.attempts, view.state_since = v.driver.actor.attempts, v.driver.actor.since
    view.position, view.forward = sim.vehicle_pose(w, v)
    view.exit = sim.successor(w, v)
    if v.motion.active { view.exit = v.motion.exit }
    view.path_progress, view.path_length = v.motion.progress, v.motion.length
    headway, preferred := f32(0.8), f32(2)
    if w.behavior.enabled { headway, preferred = v.driver.traits.headway, v.driver.traits.acceleration }
    _ = sim.following_acceleration(w, v, index, headway, preferred, &view.following)
    next := sim.forecast_vehicle(w, v, 0.1)
    view.next_body_clear = sim.body_clear(w, next, index)
    view.next_gap_clear = sim.local_gap_clear(w, next, index, 1, sim.gap_horizon(v.driver))
    view.steering, view.steering_available = replay.steering_probe(w, v.id)
    return view
}

// Text commands in, one JSON response per command out. Diagnostics go to stderr;
// no terminal prompts, display server, coordinate clicks or wall-clock sleeps.
run_session :: proc(path: string) -> int {
    s := Session{duration = 20, minimum = 0.5}
    if !load_debug_map(&s.world, path) { reply(&s, "ready", false, error = "map configuration failed"); return 2 }
    defer sim.destroy(&s.world)
    reply(&s, "ready", true)
    input: bufio.Reader
    bufio.reader_init(&input, os.to_reader(os.stdin))
    defer bufio.reader_destroy(&input)
    exit_code := 0
    for {
        line, err := bufio.reader_read_string(&input, '\n')
        quit, code := session_command(&s, strings.trim_space(line))
        delete(line)
        free_all(context.temp_allocator)
        exit_code = max(exit_code, code)
        if quit || err != nil { break }
    }
    return exit_code
}

session_command :: proc(s: ^Session, line: string) -> (quit: bool, code: int) {
    if len(line) == 0 { return }
    fields := strings.fields(line)
    defer delete(fields)
    if len(fields) == 0 { return }
    op := fields[0]
    w := &s.world
    switch op {
    case "help":
        reply(s, op, true, []string{"seek SECONDS", "step [TICKS]", "select ID", "inspect [ID]", "probe [METRES]", "list [stopped|candidates]", "save [CASE.cbor]", "load [CASE.cbor]", "check [SECONDS [MIN_METRES]]", "quit"})
    case "seek":
        seconds: f64
        ok := false
        if len(fields) == 2 { seconds, ok = strconv.parse_f64(fields[1]) }
        if !ok || !replay.finite(seconds) || seconds < 0 || seconds > 3600 { reply(s, op, false, error = "seek requires 0..3600 seconds"); return false, 2 }
        target := int(math.round(seconds/0.05))
        current := int(math.round(w.time/0.05))
        if target < current {
            if !replay.restart(w) { reply(s, op, false, error = "restart failed"); return false, 2 }
            current = 0
        }
        for _ in current..<target { sim.step(w, 0.05) }
        reply(s, op, true)
    case "step":
        ticks, ok := u64(1), true
        if len(fields) == 2 { ticks, ok = strconv.parse_u64(fields[1]) }
        if len(fields) > 2 || !ok || ticks == 0 || ticks > 20000 { reply(s, op, false, error = "step requires 1..20000 ticks"); return false, 2 }
        before, _, found := find_driver(s, s.selected)
        start: sim.Vec2
        if found { start, _ = sim.vehicle_pose(w, before) }
        for _ in 0..<int(ticks) { sim.step(w, 0.05) }
        after, _, present := find_driver(s, s.selected)
        moved: f32
        if found && present { end, _ := sim.vehicle_pose(w, after); moved = sim.vec_distance(start, end) }
        reply(s, op, true, struct { ticks: u64, moved: f32, completed: bool }{ticks, moved, found && !present})
    case "select", "inspect":
        id, ok := s.selected, true
        if len(fields) == 2 { id, ok = strconv.parse_u64(fields[1]) }
        if len(fields) > 2 || !ok || id == 0 { reply(s, op, false, error = "select/inspect requires a present driver ID"); return false, 2 }
        v, index, found := find_driver(s, id)
        if !found { reply(s, op, false, error = "driver not present; selection unchanged"); return false, 2 }
        s.selected = id
        if op == "select" { reply(s, op, true); break }
        neighbors: [dynamic]Neighbor
        defer delete(neighbors)
        p, _ := sim.vehicle_pose(w, v)
        for other in w.vehicles {
            if other.id == id { continue }
            q, g := sim.vehicle_pose(w, other)
            if sim.vec_distance(p, q) <= 5 { append(&neighbors, Neighbor{other.id, other.road, other.speed, q, g}) }
        }
        reply(s, op, true, struct { driver: Driver_View, motion: sim.Connector, neighbors: []Neighbor }{driver_view(s, v, index), v.motion, neighbors[:]})
    case "list":
        if len(fields) > 2 || (len(fields) == 2 && fields[1] != "stopped" && fields[1] != "candidates") { reply(s, op, false, error = "list accepts stopped or candidates"); return false, 2 }
        drivers: [dynamic]Driver_View
        defer delete(drivers)
        for v, index in w.vehicles {
            if len(fields) == 2 && v.speed >= 0.05 { continue }
            view := driver_view(s, v, index)
            if len(fields) == 2 && fields[1] == "candidates" && !view.steering_available { continue }
            append(&drivers, view)
        }
        reply(s, op, true, drivers[:])
    case "save", "load":
        path := strings.trim_space(strings.trim_prefix(line, op))
        if path == "" { path = "build/debug-case.cbor" }
        if op == "save" {
            _, _, selected := find_driver(s, s.selected)
            ok := selected && replay.save(w, path, s.selected, s.duration, s.minimum)
            reply(s, op, ok, path, "" if ok else "save failed or no driver selected")
            if !ok { return false, 2 }
        } else {
            id, duration, minimum, ok := replay.load_case(w, path)
            if ok { s.selected, s.duration, s.minimum = id, duration, minimum }
            reply(s, op, ok, path, "" if ok else "invalid case; current world retained")
            if !ok { return false, 2 }
        }
    case "probe":
        distance, ok := f64(0.75), true
        if len(fields) == 2 { distance, ok = strconv.parse_f64(fields[1]) }
        if len(fields) > 2 || !ok || !replay.finite(distance) || distance < 0.25 || distance > 5 { reply(s, op, false, error = "probe accepts 0.25..5 metres"); return false, 2 }
        probe, available := replay.steering_probe(w, s.selected, f32(distance))
        reply(s, op, true, struct { available: bool, probe: replay.Steering_Probe }{available, probe})
    case "check":
        duration, minimum, ok := s.duration, f64(s.minimum), true
        if len(fields) >= 2 { duration, ok = strconv.parse_f64(fields[1]) }
        if ok && len(fields) == 3 { minimum, ok = strconv.parse_f64(fields[2]) }
        if len(fields) > 3 || !ok || !replay.finite(duration) || !replay.finite(minimum) || duration <= 0 || duration > 300 || minimum <= 0 || minimum > 100 {
            reply(s, op, false, error = "check accepts seconds in (0,300] and metres in (0,100]"); return false, 2
        }
        report := replay.check(w, s.selected, duration, f32(minimum))
        reply(s, op, report.passed, report, "" if report.passed else "progress/safety assertion failed")
        if !report.passed { return false, 1 }
    case "quit":
        reply(s, op, true)
        return true, 0
    case:
        reply(s, op, false, error = "unknown command; use help")
        return false, 2
    }
    return
}
