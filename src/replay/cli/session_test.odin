package main

import "core:fmt"
import "core:encoding/json"
import "core:os"
import "core:testing"
import sim "../../sim"

@(test)
test_cli_session_controls_one_owned_world_without_a_display :: proc(t: ^testing.T) {
    s := Session{duration = 20, minimum = 0.5}
    sim.init(&s.world, []sim.Node{{{0,0}}, {{300,0}}}, []sim.Road{{0,1,8,9}}, 17)
    defer sim.destroy(&s.world)
    testing.expect(t, sim.configure(&s.world, nil, nil))
    s.world.spawn_accumulator = -1000
    driver, _ := sim.make_driver(s.world.behavior, 17, 1, .Motorcycle, .Balanced)
    append(&s.world.vehicles, sim.Vehicle{road = 0, distance = 20, speed = 2, kind = .Motorcycle, id = 1, driver = driver})
    s.world.spawned = 1
    _, code := session_command(&s, "select 1")
    testing.expect(t, code == 0 && s.selected == 1)
    _, code = session_command(&s, "step 2")
    testing.expect(t, code == 0 && s.world.time == f64(f32(0.05))*2 && s.world.vehicles[0].distance > 20)
    _, code = session_command(&s, "inspect")
    testing.expect(t, code == 0)
    time := s.world.time
    original := s.world.vehicles[0]
    _, code = session_command(&s, "check 0.1 0.01")
    testing.expect(t, code == 0 && s.world.time == time && s.world.vehicles[0] == original, "CLI check must inspect a clone")
    dir, err := os.make_directory_temp("", "traffic-debug-*", context.allocator)
    testing.expect(t, err == nil)
    if err != nil { return }
    defer delete(dir)
    defer os.remove(dir)
    path := fmt.aprintf("%s/case with spaces.cbor", dir)
    defer delete(path)
    defer os.remove(path)
    _, code = session_command(&s, fmt.tprintf("save %s", path))
    testing.expect(t, code == 0)
    _, code = session_command(&s, "step 10")
    testing.expect(t, code == 0 && s.world.time > time)
    _, code = session_command(&s, fmt.tprintf("load %s", path))
    testing.expect(t, code == 0 && s.selected == 1 && s.world.time == time && s.world.vehicles[0] == original)
    for invalid in ([]string{"seek -1", "seek nan", "seek 99999", "step 0", "step inf", "select 999", "check 0", "unknown"}) {
        _, code = session_command(&s, invalid)
        testing.expect(t, code == 2 && s.world.time == time && s.world.vehicles[0] == original && s.selected == 1)
    }
    _, code = session_command(&s, "seek 0")
    testing.expect(t, code == 0 && s.world.time == 0 && s.world.rng == 17 && s.world.behavior_seed == 17 && len(s.world.vehicles) == 0)
    quit: bool
    quit, code = session_command(&s, "quit")
    testing.expect(t, quit && code == 0)
}

@(test)
test_cli_responses_are_structured_json_not_silent_diagnostics :: proc(t: ^testing.T) {
    s: Session
    encoded, ok := encode_reply(&s, "inspect", true, struct { value: int }{42})
    testing.expect(t, ok)
    defer delete(encoded)
    parsed: struct { op: string, ok: bool, details: struct { value: int } }
    testing.expect(t, json.unmarshal(transmute([]byte)encoded, &parsed) == nil && parsed.op == "inspect" && parsed.ok && parsed.details.value == 42)
    defer delete(parsed.op)
}
