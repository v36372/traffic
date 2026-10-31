package mapdata

import "core:testing"
import "core:strings"
import "../sim"

@(test)
load_directory_rejects_without_leaking :: proc(t: ^testing.T) {
    nodes, roads, road_ok := load("src/mapdata")
    defer delete(nodes)
    defer delete(roads)
    buildings, buildings_ok := load_buildings("src/mapdata")
    defer destroy_buildings(buildings)
    signals, spawns, _, objects_ok := load_objects("src/mapdata")
    defer delete(signals)
    defer delete(spawns)
    testing.expect(t, !road_ok && !buildings_ok && !objects_ok)
}

@(test)
load_valid :: proc(t: ^testing.T) {
    nodes, roads, ok := load("src/mapdata/testdata/valid.csv")
    defer delete(nodes)
    defer delete(roads)
    testing.expect(t, ok)
    testing.expect(t, len(nodes) == 2 && len(roads) == 1)
    if ok && len(roads) == 1 {
        testing.expect(t, roads[0].from == 0 && roads[0].to == 1)
        testing.expect(t, roads[0].width == 3.5)
    }
}

@(test)
load_invalid :: proc(t: ^testing.T) {
    nodes, roads, ok := load("src/mapdata/testdata/invalid.csv")
    testing.expect(t, !ok && len(nodes) == 0 && len(roads) == 0)
    delete(nodes)
    delete(roads)
    nodes, roads, ok = load("src/mapdata/testdata/does-not-exist.csv")
    testing.expect(t, !ok && len(nodes) == 0 && len(roads) == 0)
    delete(nodes)
    delete(roads)
}

@(test)
building_validation :: proc(t: ^testing.T) {
    valid := []string{
        "B,5,1,3,0,0,0,1,1,0", // clockwise becomes CCW
        "B,1,3,5,0,0,2,0,2,2,1,1,0,2", // concave
    }
    invalid := []string{
        "B,nan,0,3,0,0,1,0,0,1", "B,0,0,3,0,0,1,0,0,1",
        "B,5,4,3,0,0,1,0,0,1", "B,5,0,2,0,0,1,0",
        "B,5,0,257", "B,5,0,3,0,0,1,0", "B,5,0,3,nan,0,1,0,0,1",
        "B,5,0,3,0,0,1,0,2,0", "B,5,0,4,0,0,2,2,0,2,2,0",
        "B,5,0,4,0,0,2,0,1,0,0,1", "B,5,0,4,0,0,1,0,1,0,0,1",
    }
    for s in valid {
        fields := strings.split(s, ",")
        buildings: [dynamic]Building
        testing.expect(t, parse_building(fields, &buildings))
        if len(buildings) > 0 { testing.expect(t, polygon_area(buildings[0].footprint) > 0) }
        destroy_buildings(buildings)
        delete(fields)
    }
    for s in invalid {
        fields := strings.split(s, ",")
        buildings: [dynamic]Building
        testing.expect(t, !parse_building(fields, &buildings))
        destroy_buildings(buildings)
        delete(fields)
    }
    buildings, ok := load_buildings("src/mapdata/testdata/buildings-invalid.csv")
    testing.expect(t, !ok && len(buildings) == 0)
    destroy_buildings(buildings)
    buildings, ok = load_buildings("src/mapdata/testdata/missing.csv")
    testing.expect(t, !ok && len(buildings) == 0)
    destroy_buildings(buildings)
    buildings, ok = load_buildings("data/hcm-buildings.csv")
    testing.expect(t, ok && len(buildings) == 123)
    for b in buildings { testing.expect(t, polygon_area(b.footprint) > 0) }
    destroy_buildings(buildings)
}

@(test)
object_validation :: proc(t: ^testing.T) {
    valid := []string{"S,0,0.05,2,1,0", "S,0,8,3,2,0,1", "P,0,1,86400,5", "D,0,0.5,1,0.4,2"}
    invalid := []string{
        "S,0,1,1,1", "S,-1,1,1,1,0", "S,0,nan,1,1,0", "S,0,1,inf,1,0",
        "S,0,0.001,1,1,0", "S,0,1,86401,1,0", "S,0,1,1,1,-1", "S,0,8,3,2,0,2", "S,0,8,3,2,0,true",
        "P,0,1,0.001,0", "P,0,1,86401,0", "P,-1,1,2,0", "P,0,-1,2,0",
        "P,0,1,2,6", "P,0,1,2,-1", "P,0,1,2,0,extra", "X,0,1,2,0",
        "D,0,0.5,1,0.4,2", "D,1,nan,1,0.4,2", "D,1,0.5,1.01,0.4,2", "D,1,0.5,1,0.3,2", "D,6,0.5,1,0.4,2",
    }
    signals: [dynamic]sim.Signal_Plan
    spawns: [dynamic]sim.Spawn_Point
    drivers := sim.default_driver_config()
    seen: [6]bool
    for s in valid {
        fields := strings.split(s, ",")
        testing.expect(t, parse_object(fields, &signals, &spawns, &drivers, &seen))
        delete(fields)
    }
    testing.expect(t, len(signals) == 2 && len(spawns) == 1 && signals[1].countdown)
    for s in invalid {
        fields := strings.split(s, ",")
        testing.expect(t, !parse_object(fields, &signals, &spawns, &drivers, &seen))
        delete(fields)
    }
    testing.expect(t, len(signals) == 2 && len(spawns) == 1)
    delete(signals); delete(spawns)
    ok: bool
    signals, spawns, _, ok = load_objects("src/mapdata/testdata/objects-invalid.csv")
    testing.expect(t, !ok && len(signals) == 0 && len(spawns) == 0)
    delete(signals); delete(spawns)
    signals, spawns, _, ok = load_objects("src/mapdata/testdata/missing.csv")
    testing.expect(t, !ok && len(signals) == 0 && len(spawns) == 0)
    delete(signals); delete(spawns)
    signals, spawns, _, ok = load_objects("src/mapdata/testdata/objects-bad-header.csv")
    testing.expect(t, !ok && len(signals) == 0 && len(spawns) == 0)
    delete(signals); delete(spawns)
    signals, spawns, _, ok = load_objects("src/mapdata/testdata/objects-valid.csv")
    testing.expect(t, ok && len(signals) == 1 && len(spawns) == 2)
    delete(signals); delete(spawns)
}
