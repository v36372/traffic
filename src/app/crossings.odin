package app

import "core:fmt"
import "core:math"
import sim "../sim"

CROSSING_PAINT_HEIGHT :: f32(0.055)
CROSSING_LANDING_WIDTH :: f32(0.4) // Within sim's 0.45m full-span clearance margin.

// Visual paint only: no pedestrian actors, yielding, or pedestrian safety model.
// The sim owns band/span, STOP and TURN entry. No render-only bar displacement.
cache_crosswalks :: proc(scene: ^Road_Scene, world: ^sim.World) {
    for plan in world.signals {
        for road, i in world.roads {
            if road.to != plan.node { continue }
            near, far, center_lateral, width, ok := sim.crossing_band(world, i)
            if !ok {
                fmt.eprintf("Crossing paint omitted: unsupported controlled approach %d\n", i)
                continue
            }
            start, forward, left, _ := road_frame(world, road)
            // center_lateral is AXIS-relative and already spans both directions.
            a := start+forward*near+left*center_lateral+Vec3{0, CROSSING_PAINT_HEIGHT, 0}
            b := start+forward*far+left*center_lateral+Vec3{0, CROSSING_PAINT_HEIGHT, 0}
            // 0.5m white bars at 1m pitch, centered within the full street span.
            count := math.max(1, int(math.floor(width+0.5)))
            bar_width := math.min(0.5, width)
            for stripe in 0..<count {
                shift := left*(f32(stripe)-f32(count-1)*0.5)
                cache_strip(&scene.crosswalks, a+shift, b+shift, left, bar_width)
            }
            // Interrupt just the curb with a flush sidewalk-colored landing;
            // never add pavement or expand the carriageway into the sidewalk.
            for side in ([]f32{-1, 1}) {
                shift := left*(side*(width+CROSSING_LANDING_WIDTH)*0.5)
                cache_strip(&scene.landings, a+shift, b+shift, left, CROSSING_LANDING_WIDTH)
            }
        }
    }
}
