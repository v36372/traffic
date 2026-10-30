package sim

import "core:math"
import "core:slice"

pavement_turn :: proc(a, b, c: Vec2) -> f32 {
    return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)
}

// Same mouth+endpoint convex hull for simulation fitting and rendered pavement.
// extra is TOTAL strip width addition: 0 asphalt, 0.4 curb, 4.4 sidewalk.
// East/north CCW; caller owns/deletes the returned polygon.
junction_polygon :: proc(w: ^World, node: int, extra: f32 = 0) -> [dynamic]Vec2 {
    points: [dynamic]Vec2
    defer delete(points)
    for road, i in w.roads {
        if road.from != node && road.to != node { continue }
        length := road_length(w, i)
        if length <= 0 { continue }
        a, b := w.nodes[road.from].position, w.nodes[road.to].position
        f := Vec2{(b.x-a.x)/length, (b.y-a.y)/length}
        left := Vec2{-f.y, f.x}
        depth := junction_mouth_depth(w, i)
        along, endpoint_along := depth, f32(0)
        if road.to == node { along, endpoint_along = length-depth, length }
        offset, half := road_offset(w, i), (road.width+extra)*0.5
        mouth := Vec2{a.x+f.x*along+left.x*offset, a.y+f.y*along+left.y*offset}
        endpoint := Vec2{a.x+f.x*endpoint_along+left.x*offset, a.y+f.y*endpoint_along+left.y*offset}
        side := Vec2{left.x*half, left.y*half}
        append(&points, Vec2{mouth.x-side.x,mouth.y-side.y}, Vec2{mouth.x+side.x,mouth.y+side.y},
            Vec2{endpoint.x-side.x,endpoint.y-side.y}, Vec2{endpoint.x+side.x,endpoint.y+side.y})
    }
    // Preserve the renderer's previous east/-north sorting and CCW winding.
    slice.sort_by(points[:], proc(a, b: Vec2) -> bool { return a.x < b.x || (a.x == b.x && a.y > b.y) })
    hull: [dynamic]Vec2
    for p in points {
        for len(hull) >= 2 && pavement_turn(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 { resize(&hull, len(hull)-1) }
        append(&hull, p)
    }
    lower := len(hull)
    for i := len(points)-2; i >= 0; i -= 1 {
        p := points[i]
        for len(hull) > lower && pavement_turn(hull[len(hull)-2], hull[len(hull)-1], p) <= 0 { resize(&hull, len(hull)-1) }
        append(&hull, p)
    }
    if len(hull) > 0 { resize(&hull, len(hull)-1) }
    return hull
}

point_on_junction_pavement :: proc(w: ^World, node: int, point: Vec2, hull: []Vec2) -> bool {
    if len(hull) >= 3 {
        inside := true
        for a, i in hull {
            if pavement_turn(a, hull[(i+1)%len(hull)], point) < -0.0001 { inside = false; break }
        }
        if inside { return true }
    }
    for road, i in w.roads {
        if road.from != node && road.to != node { continue }
        length := road_length(w, i)
        if length <= 0 { continue }
        a, b := w.nodes[road.from].position, w.nodes[road.to].position
        f := Vec2{(b.x-a.x)/length, (b.y-a.y)/length}
        along := (point.x-a.x)*f.x+(point.y-a.y)*f.y
        side := -(point.x-a.x)*f.y+(point.y-a.y)*f.x-road_offset(w, i)
        if along >= -0.0001 && along <= length+0.0001 && abs(side) <= road.width*0.5+0.0001 { return true }
    }
    return false
}

// Bounded gameplay fitting, not continuous swept-volume certification. Match
// the independent app oracle's corners+edge centers, sampled at <=0.05m.
connector_on_pavement :: proc(w: ^World, c: Connector, kind: Vehicle_Kind, hull: []Vec2) -> bool {
    samples := max(1, int(math.ceil(c.length/0.05)))
    for k in 0..=samples {
        p, f := connector_pose(c, c.length*f32(k)/f32(samples))
        for along in ([]f32{-0.5,0,0.5}) {
            for across in ([]f32{-0.5,0,0.5}) {
                if along == 0 && across == 0 { continue }
                x, y := along*kind_length(kind), across*kind_width(kind)
                point := Vec2{p.x+f.x*x-f.y*y, p.y+f.y*x+f.x*y}
                if !point_on_junction_pavement(w, c.node, point, hull) { return false }
            }
        }
    }
    return true
}
