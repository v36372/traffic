package app

import "core:math"
import sim "../sim"
import rl "vendor:raylib"

// Sidewalk, curb, pavement and markings. Flat layers avoid z-fighting; all
// topology/hulls are computed on load, never in the frame loop.
Road_Scene :: struct {
    layers: [4][dynamic][3]Vec3,
    junction_nodes: [dynamic]int,
    crosswalks, landings: [dynamic][3]Vec3,
}

ROAD_EXTRA :: [3]f32{4.4, 0.4, 0}
@(rodata) ROAD_HEIGHT := [4]f32{0, 0.015, 0.03, 0.045}
@(rodata) ROAD_COLOR := [4]rl.Color{{179, 177, 164, 255}, {218, 214, 194, 255}, ASPHALT, {200, 193, 160, 255}}

destroy_road_scene :: proc(scene: ^Road_Scene) {
    for &layer in scene.layers { delete(layer) }
    delete(scene.junction_nodes)
    delete(scene.crosswalks)
    delete(scene.landings)
    scene^ = {}
}

cache_strip :: proc(triangles: ^[dynamic][3]Vec3, a, b, left: Vec3, width: f32) {
    side := left*(width*0.5)
    append(triangles, [3]Vec3{a-side, b-side, b+side}, [3]Vec3{a-side, b+side, a+side})
}

road_turn :: proc(a, b, c: Vec3) -> f32 {
    // Positive means upward-facing after east/north -> raylib projection.
    return (b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2])
}

build_road_scene :: proc(world: ^sim.World, offsets: []f32) -> Road_Scene {
    scene: Road_Scene
    for road, i in world.roads {
        start, forward, left, length := road_frame(world, road)
        if length <= 0 { continue }
        for extra, layer in ROAD_EXTRA {
            a := start+left*offsets[i]+Vec3{0, ROAD_HEIGHT[layer], 0}
            cache_strip(&scene.layers[layer], a, a+forward*length, left, road.width+extra)
        }
        if length < 12 { continue }
        center := start+left*offsets[i]+forward*(length*0.55)+Vec3{0, ROAD_HEIGHT[3], 0}
        size := math.min(road.width*0.18, 0.7)
        append(&scene.layers[3], [3]Vec3{center+forward*1.5, center+left*size, center-forward*0.4}, [3]Vec3{center+forward*1.5, center-forward*0.4, center-left*size})
        // A reciprocal pair provides an actual two-way divider, not a lane count.
        for other, j in world.roads {
            if j <= i || other.from != road.to || other.to != road.from { continue }
            for d: f32 = 10; d < length-8; d += 7 {
                a := start+forward*d+Vec3{0, ROAD_HEIGHT[3], 0}
                cache_strip(&scene.layers[3], a, a+forward*math.min(2.5, length-8-d), left, 0.12)
            }
            break
        }
    }
    // O(nodes*roads) at load is enough for Phase A. Index incident edges before
    // district-scale use. Only shared IDs join: geometric crossings never do.
    for _, node in world.nodes {
        neighbors := make(map[int]bool)
        for road in world.roads {
            if road.from == node && road.to != node { neighbors[road.to] = true }
            if road.to == node && road.from != node { neighbors[road.from] = true }
        }
        shared := len(neighbors) >= 2
        delete(neighbors)
        if !shared { continue }
        append(&scene.junction_nodes, node)
        for extra, layer in ROAD_EXTRA {
            // The simulation fits bodies against this exact polygon; rendering
            // only triangulates/projects it. No second hull or pavement sizing.
            hull := sim.junction_polygon(world, node, extra)
            if len(hull) >= 3 {
                for i in 1..<len(hull)-1 {
                    a, b, c := hull[0], hull[i], hull[i+1]
                    y := ROAD_HEIGHT[layer]
                    append(&scene.layers[layer], [3]Vec3{{a.x, y, -a.y}, {b.x, y, -b.y}, {c.x, y, -c.y}})
                }
            }
            delete(hull)
        }
    }
    cache_crosswalks(&scene, world)
    return scene
}

draw_road_scene :: proc(scene: ^Road_Scene) {
    for layer, i in scene.layers {
        for tri in layer { rl.DrawTriangle3D(tri[0], tri[1], tri[2], ROAD_COLOR[i]) }
    }
    for tri in scene.crosswalks { rl.DrawTriangle3D(tri[0], tri[1], tri[2], PAPER) }
    for tri in scene.landings { rl.DrawTriangle3D(tri[0], tri[1], tri[2], ROAD_COLOR[0]) }
}
