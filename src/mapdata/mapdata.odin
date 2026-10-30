package mapdata

import "core:os"
import "core:strings"
import "core:strconv"
import "../sim"

// Caller owns both arrays on success. Invalid inputs return empty arrays.
load :: proc(path: string) -> (nodes: [dynamic]sim.Node, roads: [dynamic]sim.Road, ok: bool) {
    nodes, roads, ok = load_inner(path)
    if !ok { delete(nodes); delete(roads); return nil, nil, false }
    return
}

load_inner :: proc(path: string) -> (nodes: [dynamic]sim.Node, roads: [dynamic]sim.Road, ok: bool) {
    data, err := os.read_entire_file(path, context.allocator)
    if err != nil { return }
    defer delete(data)
    lines := strings.split_lines(string(data))
    defer delete(lines)
    if len(lines) == 0 || lines[0] != "HCM_MAP,1" { return }
    edges_started := false
    for line in lines[1:] {
        if line == "" { continue }
        fields := strings.split(line, ",")
        defer delete(fields)
        if len(fields) == 3 && fields[0] == "N" && !edges_started {
            x, xok := strconv.parse_f32(fields[1])
            y, yok := strconv.parse_f32(fields[2])
            if !xok || !yok || !(x > -1e7 && x < 1e7 && y > -1e7 && y < 1e7) { return }
            append(&nodes, sim.Node{position = {x, y}})
        } else if len(fields) == 5 && fields[0] == "E" {
            edges_started = true
            a, aok := strconv.parse_int(fields[1], 10)
            b, bok := strconv.parse_int(fields[2], 10)
            width, wok := strconv.parse_f32(fields[3])
            speed, sok := strconv.parse_f32(fields[4])
            if !aok || !bok || !wok || !sok || a < 0 || b < 0 || a >= len(nodes) || b >= len(nodes) || a == b || !(width > 0 && width < 100 && speed > 0 && speed < 100) { return }
            if nodes[a].position == nodes[b].position { return }
            append(&roads, sim.Road{from = a, to = b, width = width, speed_limit = speed})
        } else { return }
    }
    ok = len(nodes) > 0 && len(roads) > 0
    return
}
