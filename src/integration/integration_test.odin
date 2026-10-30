package integration

import "core:fmt"
import "core:testing"
import "../sim"
import "../mapdata"

check_world :: proc(t: ^testing.T, w: ^sim.World) {
    testing.expect(t, w.spawned == w.completed + len(w.vehicles), "trip accounting must be conserved")
    for v in w.vehicles {
        valid := v.road >= 0 && v.road < len(w.roads)
        testing.expect(t, valid, "vehicle road reference must be valid")
        if !valid { continue }
        testing.expect(t, v.distance >= 0 && v.distance < 1e8, "distance must be finite and nonnegative")
        testing.expect(t, v.speed >= 0 && v.speed < 100, "speed must be finite and bounded")
        testing.expect(t, abs(v.lateral) <= w.roads[v.road].width * 0.5, "center must stay on carriageway")
    }
}

@(test)
demo_survives_two_minutes :: proc(t: ^testing.T) {
    w: sim.World
    sim.init_demo(&w, 42)
    defer sim.destroy(&w)
    for _ in 0..<2400 {
        sim.step(&w, 0.05)
        check_world(t, &w)
    }
    testing.expect(t, w.spawned > 0, "demo must generate visible traffic")
    testing.expect(t, w.completed > 0, "traffic must finish trips")
    fmt.printf("Demo: spawned=%d completed=%d active=%d\n", w.spawned, w.completed, len(w.vehicles))
}

@(test)
real_map_loads_and_runs :: proc(t: ^testing.T) {
    nodes, roads, ok := mapdata.load("data/hcm.csv")
    testing.expect(t, ok, "run from repository root with baked data/hcm.csv")
    if !ok { return }
    defer delete(nodes)
    defer delete(roads)
    testing.expect(t, len(nodes) > 0 && len(roads) > 0)
    w: sim.World
    sim.init(&w, nodes[:], roads[:], 42)
    defer sim.destroy(&w)
    for _ in 0..<1200 {
        sim.step(&w, 0.05)
        check_world(t, &w)
    }
    testing.expect(t, w.spawned > 0, "real geometry must admit traffic")
    fmt.printf("HCMC: nodes=%d roads=%d spawned=%d completed=%d active=%d\n", len(nodes), len(roads), w.spawned, w.completed, len(w.vehicles))
}

@(test)
real_buildings_preserve_skyline :: proc(t: ^testing.T) {
    buildings, ok := mapdata.load_buildings("data/hcm-buildings.csv")
    testing.expect(t, ok, "real-map building sidecar must load")
    if !ok { return }
    defer mapdata.destroy_buildings(buildings)
    testing.expect(t, len(buildings) > 0)
    tallest: f32
    towers := 0
    for building in buildings {
        testing.expect(t, len(building.footprint) >= 3)
        testing.expect(t, building.height > 0 && building.height < 1000)
        tallest = max(tallest, building.height)
        if building.height >= 60 { towers += 1 }
    }
    testing.expect(t, tallest == 193, "preserve the source-tagged Saigon Centre height")
    testing.expect(t, towers >= 2, "source includes multiple tall buildings, not uniform blocks")
    fmt.printf("HCMC buildings: count=%d towers=%d tallest=%.1fm\n", len(buildings), towers, tallest)
}

@(test)
authored_maps_use_the_same_runtime :: proc(t: ^testing.T) {
    for name in ([]string{"intersection", "bottleneck", "downtown", "showcase"}) {
        path := fmt.aprintf("build/maps/%s.csv", name)
        building_path := fmt.aprintf("build/maps/%s-buildings.csv", name)
        objects_path := fmt.aprintf("build/maps/%s-objects.csv", name)
        nodes, roads, ok := mapdata.load(path)
        buildings, buildings_ok := mapdata.load_buildings(building_path)
        signals, spawns, objects_ok := mapdata.load_objects(objects_path)
        delete(path)
        delete(building_path)
        delete(objects_path)
        testing.expect(t, ok && buildings_ok && objects_ok, "make maps must compile valid runtime files")
        if ok {
            world: sim.World
            sim.init(&world, nodes[:], roads[:], 42)
            configured := sim.configure(&world, signals[:], spawns[:])
            testing.expect(t, configured, "compiled objects must pass runtime topology validation")
            if name == "intersection" {
                testing.expect(t, len(world.signals) > 0 && len(world.spawns) > 0, "intersection exercises operational objects")
            }
            for _ in 0..<2400 {
                sim.step(&world, 0.05)
                check_world(t, &world)
                if len(spawns) > 0 {
                    testing.expect(t, world.requested == world.spawned + world.pending, "blocked scheduled demand must not disappear")
                }
            }
            if len(spawns) > 0 {
                testing.expect(t, world.completed > 0, "configured trips must reach their destinations")
            }
            testing.expect(t, world.spawned > 0, "authored streets must admit traffic")
            fmt.printf("Authored %s: nodes=%d roads=%d buildings=%d spawned=%d\n", name, len(nodes), len(roads), len(buildings), world.spawned)
            sim.destroy(&world)
        }
        delete(nodes)
        delete(roads)
        delete(signals)
        delete(spawns)
        mapdata.destroy_buildings(buildings)
    }
}
