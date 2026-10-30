package mapdata

import "core:os"
import "core:strings"
import "core:strconv"
import "../sim"

// Caller owns both flat arrays on success; configure copies them.
load_objects :: proc(path: string) -> (signals: [dynamic]sim.Signal_Plan, spawns: [dynamic]sim.Spawn_Point, drivers: sim.Driver_Config, ok: bool) {
    signals, spawns, drivers, ok = load_objects_inner(path)
    if !ok { delete(signals); delete(spawns); return nil, nil, {}, false }
    return
}

load_objects_inner :: proc(path: string) -> (signals: [dynamic]sim.Signal_Plan, spawns: [dynamic]sim.Spawn_Point, drivers: sim.Driver_Config, ok: bool) {
    data, err := os.read_entire_file(path, context.allocator)
    defer delete(data)
    if err != nil { return }
    lines := strings.split_lines(string(data))
    defer delete(lines)
    if len(lines) == 0 || lines[0] != "TRAFFIC_OBJECTS,1" { return }
    drivers = sim.default_driver_config()
    seen: [6]bool
    for line in lines[1:] {
        if line == "" { continue }
        fields := strings.split(line, ",")
        valid := parse_object(fields, &signals, &spawns, &drivers, &seen)
        delete(fields)
        if !valid { return }
    }
    ok = true
    return
}

parse_object :: proc(fields: []string, signals: ^[dynamic]sim.Signal_Plan, spawns: ^[dynamic]sim.Spawn_Point, drivers: ^sim.Driver_Config, seen: ^[6]bool) -> bool {
    if len(fields) == 6 && fields[0] == "D" {
        kind, kok := strconv.parse_int(fields[1], 10)
        a, aok := strconv.parse_f32(fields[2])
        b, bok := strconv.parse_f32(fields[3])
        c, cok := strconv.parse_f32(fields[4])
        d, dok := strconv.parse_f32(fields[5])
        if !kok || !aok || !bok || !cok || !dok || kind < 0 || kind > 5 { return false }
        if seen[kind] || !(a >= 0.5 && a <= b && b <= 1 && c >= 0.4 && c <= d && d <= 2) { return false }
        seen[kind] = true
        drivers.enabled = true
        drivers.classes[kind] = {a, b, c, d}
        return true
    }
    if len(fields) == 6 && fields[0] == "S" {
        node, nok := strconv.parse_int(fields[1], 10)
        green, gok := strconv.parse_f32(fields[2])
        amber, aok := strconv.parse_f32(fields[3])
        clearance, cok := strconv.parse_f32(fields[4])
        offset, ook := strconv.parse_f32(fields[5])
        if !nok || !gok || !aok || !cok || !ook || node < 0 || !(green >= 0.05 && green <= 86400 && amber >= 0.05 && amber <= 86400 && clearance >= 0.05 && clearance <= 86400 && offset >= 0 && offset <= 86400) { return false }
        append(signals, sim.Signal_Plan{node = node, green = green, amber = amber, clearance = clearance, offset = offset})
        return true
    }
    if len(fields) == 5 && fields[0] == "P" {
        road, rok := strconv.parse_int(fields[1], 10)
        destination, dok := strconv.parse_int(fields[2], 10)
        interval, iok := strconv.parse_f32(fields[3])
        kind, kok := strconv.parse_int(fields[4], 10)
        if !rok || !dok || !iok || !kok || road < 0 || destination < 0 || !(interval >= 0.05 && interval <= 86400) || kind < 0 || kind > 5 { return false }
        append(spawns, sim.Spawn_Point{road = road, destination = destination, interval = interval, kind = sim.Vehicle_Kind(kind)})
        return true
    }
    return false
}
