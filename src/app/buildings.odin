package app

import "core:fmt"
import "core:math"
import "core:path/filepath"
import "core:strings"
import mapdata "../mapdata"
import sim "../sim"
import rl "vendor:raylib"

Building_Triangle :: struct {
    a, b, c: Vec3,
    color: rl.Color,
}

// CPU geometry is cached once per scene; DrawTriangle3D uses raylib's batch.
// No GPU ownership means reset/exit cleanup also works without a graphics context.
Building_Scene :: struct {
    triangles: [dynamic]Building_Triangle,
    count: int,
}

building_sidecar :: proc(path: string) -> string {
    return fmt.aprintf("%s-buildings.csv", strings.trim_suffix(path, filepath.ext(path)))
}

destroy_building_scene :: proc(scene: ^Building_Scene) {
    delete(scene.triangles)
    scene^ = {}
}

load_building_scene :: proc(map_path: string) -> Building_Scene {
    path := building_sidecar(map_path)
    defer delete(path)
    buildings, ok := mapdata.load_buildings(path)
    defer mapdata.destroy_buildings(buildings)
    if !ok {
        fmt.printf("Buildings unavailable: %s; traffic remains available\n", path)
        return {}
    }
    scene := build_building_scene(buildings[:])
    fmt.printf("Buildings loaded: %s (%d/%d footprints, %d cached triangles)\n", path, scene.count, len(buildings), len(scene.triangles))
    return scene
}

turn_2d :: proc(a, b, c: sim.Vec2) -> f64 {
    return (f64(b.x)-f64(a.x))*(f64(c.y)-f64(a.y)) - (f64(b.y)-f64(a.y))*(f64(c.x)-f64(a.x))
}

// Ear clipping supports concave simple outlines, not holes (the sidecar contract).
// O(n³) worst-case per footprint (loader caps 256 vertices), once at scene load;
// use an indexed tessellator for city-scale data.
triangulate_roof :: proc(points: []sim.Vec2) -> (triangles: [dynamic][3]int, ok: bool) {
    if len(points) < 3 { return }
    ring := make([dynamic]int, 0, len(points))
    defer delete(ring)
    area: f64
    for p, i in points {
        q := points[(i+1)%len(points)]
        area += f64(p.x)*f64(q.y)-f64(q.x)*f64(p.y)
    }
    if math.abs(area) < 1e-8 { return }
    for i in 0..<len(points) {
        if area > 0 { append(&ring, i) } else { append(&ring, len(points)-1-i) }
    }
    for len(ring) > 3 {
        clipped := false
        for b, i in ring {
            a, c := ring[(i+len(ring)-1)%len(ring)], ring[(i+1)%len(ring)]
            cross := turn_2d(points[a], points[b], points[c])
            if cross < -1e-8 { continue }
            // Collinear source points don't need a zero-area roof triangle.
            blocked := false
            if cross > 1e-8 {
                for p in ring {
                    if p == a || p == b || p == c { continue }
                    if turn_2d(points[a], points[b], points[p]) >= -1e-8 &&
                       turn_2d(points[b], points[c], points[p]) >= -1e-8 &&
                       turn_2d(points[c], points[a], points[p]) >= -1e-8 {
                        blocked = true; break
                    }
                }
            }
            if blocked { continue }
            if cross > 1e-8 { append(&triangles, [3]int{a, b, c}) }
            for j := i; j < len(ring)-1; j += 1 { ring[j] = ring[j+1] }
            resize(&ring, len(ring)-1)
            clipped = true
            break
        }
        if !clipped { delete(triangles); return nil, false }
    }
    if turn_2d(points[ring[0]], points[ring[1]], points[ring[2]]) > 1e-8 {
        append(&triangles, [3]int{ring[0], ring[1], ring[2]})
    }
    ok = len(triangles) > 0
    return
}

building_material :: proc(kind: mapdata.Building_Kind) -> rl.Color {
    switch kind {
    case .Lowrise: return {183, 170, 148, 255}
    case .Apartment: return {191, 187, 170, 255}
    case .Office: return {134, 157, 163, 255}
    case .Landmark: return {181, 190, 174, 255}
    }
    return PAPER
}

shade_building :: proc(base: rl.Color, factor: f32) -> rl.Color {
    return {u8(math.clamp(f32(base.r)*factor, 0, 255)), u8(math.clamp(f32(base.g)*factor, 0, 255)), u8(math.clamp(f32(base.b)*factor, 0, 255)), 255}
}

building_vertex :: proc(p: sim.Vec2, y: f32) -> Vec3 { return {p.x, y, -p.y} }

extrude_building :: proc(scene: ^Building_Scene, polygon: []sim.Vec2, roof: [][3]int, bottom, top: f32, material: rl.Color) {
    for p, i in polygon {
        q := polygon[(i+1)%len(polygon)]
        dx, dy := q.x-p.x, q.y-p.y
        length := math.sqrt(dx*dx+dy*dy)
        if length < 1e-5 { continue }
        // CCW outward east/north normal = (dy,-dx); fixed NW light, no textures.
        shade := 0.80 + 0.13*(-0.6*dy-0.8*dx)/length
        color := shade_building(material, shade)
        a, b := building_vertex(p, bottom), building_vertex(q, bottom)
        c, d := building_vertex(q, top), building_vertex(p, top)
        append(&scene.triangles, Building_Triangle{a, b, c, color}, Building_Triangle{a, c, d, color})
    }
    for tri in roof {
        append(&scene.triangles, Building_Triangle{building_vertex(polygon[tri[0]], top), building_vertex(polygon[tri[1]], top), building_vertex(polygon[tri[2]], top), shade_building(material, 1.04)})
    }
}

build_building_scene :: proc(buildings: []mapdata.Building) -> Building_Scene {
    scene: Building_Scene
    for building in buildings {
        polygon := building.footprint
        roof, ok := triangulate_roof(polygon)
        defer delete(roof)
        if !ok { continue }
        material := building_material(building.kind)
        // Only tall convex tower/mixed-use footprints get a modest podium.
        // Homothetic inset stays inside a convex outline; never use bounding boxes
        // or inset concave polygons, which could cover roads or courtyards.
        convex := true
        low, high, center := polygon[0], polygon[0], sim.Vec2{}
        for p, i in polygon {
            if turn_2d(p, polygon[(i+1)%len(polygon)], polygon[(i+2)%len(polygon)]) < -1e-8 { convex = false }
            low.x = math.min(low.x, p.x); low.y = math.min(low.y, p.y)
            high.x = math.max(high.x, p.x); high.y = math.max(high.y, p.y)
            center.x += p.x; center.y += p.y
        }
        center.x /= f32(len(polygon)); center.y /= f32(len(polygon))
        if building.kind == .Office && building.height >= 30 && convex && math.min(high.x-low.x, high.y-low.y) >= 12 {
            podium := math.min(12, building.height*0.18)
            extrude_building(&scene, polygon, roof[:], 0, podium, material)
            inset := make([]sim.Vec2, len(polygon))
            defer delete(inset)
            for p, i in polygon { inset[i] = {center.x+(p.x-center.x)*0.88, center.y+(p.y-center.y)*0.88} }
            extrude_building(&scene, inset, roof[:], podium, building.height, material)
        } else {
            // Lowrise, apartment slabs and broad civic forms keep their real outline
            // and flat roof. The data, not invented ornament, supplies proportions.
            extrude_building(&scene, polygon, roof[:], 0, building.height, material)
        }
        scene.count += 1
    }
    return scene
}

fit_scene_view :: proc(world: ^sim.World, scene: ^Building_Scene, aspect: f32 = 1.6) -> View {
    low, high := road_bounds(world)
    for tri in scene.triangles {
        for p in ([3]Vec3{tri.a, tri.b, tri.c}) {
            for axis in 0..<3 {
                low[axis] = math.min(low[axis], p[axis])
                high[axis] = math.max(high[axis], p[axis])
            }
        }
    }
    return fit_bounds_view(low, high, aspect)
}

draw_buildings :: proc(scene: ^Building_Scene) {
    for tri in scene.triangles { rl.DrawTriangle3D(tri.a, tri.b, tri.c, tri.color) }
}
