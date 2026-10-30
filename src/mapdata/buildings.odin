package mapdata

import "core:os"
import "core:strings"
import "core:strconv"
import "../sim"

Building_Kind :: enum { Lowrise, Apartment, Office, Landmark }
Building :: struct { footprint: []sim.Vec2, height: f32, kind: Building_Kind }

// Caller owns the returned footprints and array, and must destroy them together.
destroy_buildings :: proc(buildings: [dynamic]Building) {
    for b in buildings { delete(b.footprint) }
    delete(buildings)
}

load_buildings :: proc(path: string) -> (buildings: [dynamic]Building, ok: bool) {
    buildings, ok = load_buildings_inner(path)
    if !ok { destroy_buildings(buildings); return nil, false }
    return
}

load_buildings_inner :: proc(path: string) -> (buildings: [dynamic]Building, ok: bool) {
    data, err := os.read_entire_file(path, context.allocator)
    defer delete(data)
    if err != nil { return }
    lines := strings.split_lines(string(data))
    defer delete(lines)
    if len(lines) == 0 || lines[0] != "HCM_BUILDINGS,1" { return }
    for line in lines[1:] {
        if line == "" { continue }
        fields := strings.split(line, ",")
        valid := parse_building(fields, &buildings)
        delete(fields)
        if !valid { return }
    }
    ok = true
    return
}

parse_building :: proc(fields: []string, buildings: ^[dynamic]Building) -> bool {
    if len(fields) < 4 || fields[0] != "B" { return false }
    height, hok := strconv.parse_f32(fields[1])
    kind, kok := strconv.parse_int(fields[2], 10)
    count, cok := strconv.parse_int(fields[3], 10)
    if !hok || !kok || !cok || !(height > 0 && height < 1000) || kind < 0 || kind > 3 || count < 3 || count > 256 || len(fields) != 4 + count * 2 { return false }
    points := make([]sim.Vec2, count)
    for i in 0..<count {
        x, xok := strconv.parse_f32(fields[4 + 2*i])
        y, yok := strconv.parse_f32(fields[5 + 2*i])
        if !xok || !yok || !(x > -1e7 && x < 1e7 && y > -1e7 && y < 1e7) { delete(points); return false }
        points[i] = {x, y}
    }
    area := polygon_area(points)
    if area == 0 || !polygon_simple(points) { delete(points); return false }
    if area < 0 {
        for i in 0..<len(points)/2 {
            points[i], points[len(points)-1-i] = points[len(points)-1-i], points[i]
        }
    }
    append(buildings, Building{footprint = points, height = height, kind = Building_Kind(kind)})
    return true
}

cross :: proc(a, b, c: sim.Vec2) -> f64 {
    return (f64(b.x)-f64(a.x))*(f64(c.y)-f64(a.y)) - (f64(b.y)-f64(a.y))*(f64(c.x)-f64(a.x))
}
polygon_area :: proc(p: []sim.Vec2) -> f64 {
    area: f64
    for i in 0..<len(p) {
        a, b := p[i], p[(i+1)%len(p)]
        area += f64(a.x)*f64(b.y) - f64(b.x)*f64(a.y)
    }
    return area
}
on_segment :: proc(a, b, p: sim.Vec2) -> bool {
    return p.x >= min(a.x,b.x) && p.x <= max(a.x,b.x) && p.y >= min(a.y,b.y) && p.y <= max(a.y,b.y)
}
segments_intersect :: proc(a, b, c, d: sim.Vec2) -> bool {
    x, y, z, w := cross(a,b,c), cross(a,b,d), cross(c,d,a), cross(c,d,b)
    return (x == 0 && on_segment(a,b,c)) || (y == 0 && on_segment(a,b,d)) || (z == 0 && on_segment(c,d,a)) || (w == 0 && on_segment(c,d,b)) || (x*y < 0 && z*w < 0)
}
polygon_simple :: proc(p: []sim.Vec2) -> bool {
    n := len(p)
    for i in 0..<n {
        a, b, c := p[(i+n-1)%n], p[i], p[(i+1)%n]
        if a == b { return false }
        if cross(a,b,c) == 0 && (f64(a.x)-f64(b.x))*(f64(c.x)-f64(b.x)) + (f64(a.y)-f64(b.y))*(f64(c.y)-f64(b.y)) > 0 { return false }
        for j in i+1..<n {
            if j == i+1 || (i == 0 && j == n-1) { continue }
            if segments_intersect(p[i], p[(i+1)%n], p[j], p[(j+1)%n]) { return false }
        }
    }
    return true
}
