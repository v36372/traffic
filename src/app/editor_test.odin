package app

import "core:math"
import "core:testing"
import sim "../sim"

@(test)
test_editor_selection_and_bounded_exact_seek :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0, 0}}, {{100, 0}}}, []sim.Road{{0, 1, 7, 9}})
    defer sim.destroy(&world)
    append(&world.vehicles, sim.Vehicle{id = 11, road = 0, kind = .Car})
    append(&world.vehicles, sim.Vehicle{id = 12, road = 0, kind = .Car})
    editor := editor_init()
    editor_select_next(&editor, &world)
    testing.expect(t, editor.selected == 11)
    editor_select_next(&editor, &world)
    testing.expect(t, editor.selected == 12)
    poses := make(Render_Poses)
    defer delete(poses)
    editor.run_to, editor.seeking = 0.15, true
    editor_seek_frame(&editor, &world, &poses)
    testing.expect(t, math.abs(world.time-0.15) < 0.000001 && !editor.seeking)
    editor.run_to, editor.seeking = 0.05, true
    editor_seek_frame(&editor, &world, &poses)
    testing.expect(t, math.abs(world.time-0.15) < 0.000001 && !editor.seeking)
}

@(test)
test_editor_cross_road_blockers_and_typed_seek_target :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world,
        []sim.Node{{{-10, 0}}, {{10, 0}}, {{0, -10}}, {{0, 10}}},
        []sim.Road{{0, 1, 8, 9}, {2, 3, 8, 9}})
    defer sim.destroy(&world)
    selected := sim.Vehicle{id = 1, road = 0, distance = 10, speed = 0, kind = .Car}
    crossing := sim.Vehicle{id = 2, road = 1, distance = 10, speed = 0, kind = .Car}
    append(&world.vehicles, selected, crossing)
    testing.expect(t, editor_is_blocker(&world, selected, crossing))

    editor := editor_init()
    copy(editor.run_to_text[:], "12.126")
    editor.run_to_len = 6
    target, valid := editor_parse_target(&editor)
    testing.expect(t, valid && math.abs(target-12.15) < 0.000001)
    copy(editor.run_to_text[:], "3600.1")
    editor.run_to_len = 6
    _, valid = editor_parse_target(&editor)
    testing.expect(t, !valid)
}

@(test)
test_editor_check_result_is_compact_and_two_line :: proc(t: ^testing.T) {
    editor := editor_init()
    editor_set_check_result(&editor, false, 0, 0, 0)
    testing.expect(t, string(editor.result[0][:editor.result_len[0]]) == "FAIL progress 0.00m")
    testing.expect(t, string(editor.result[1][:editor.result_len[1]]) == "overlap 0 / off-road 0")
    editor_set_result(&editor, "Saved build/debug-case.cbor")
    testing.expect(t, editor.result_len[1] == 0)
}

@(test)
test_editor_reset_preserves_loaded_replay_seed :: proc(t: ^testing.T) {
    world: sim.World
    sim.init(&world, []sim.Node{{{0,0}}, {{100,0}}}, []sim.Road{{0,1,8,9}}, 17)
    defer sim.destroy(&world)
    sim.step(&world, 0.05)
    testing.expect(t, reset_world(&world))
    testing.expect(t, world.time == 0 && world.behavior_seed == 17 && world.rng == 17)
}
