package app

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import sim "../sim"
import rl "vendor:raylib"

CAPTURE_FPS :: 60
SEQUENCE_FRAMES :: 600
CAPTURE_EVERY :: 6

Frontend_Options :: struct {
    map_path: string,
    screenshot, sequence_dir: string,
    smoke: bool,
    demand_scale: f32,
    warmup: f64, // -1 selects the existing map/smoke default.
}

parse_options :: proc(args: []string) -> (options: Frontend_Options, ok: bool) {
    options = {map_path = DEFAULT_MAP_PATH, demand_scale = 1, warmup = -1}
    for i := 0; i < len(args); i += 1 {
        arg := args[i]
        if arg == "--smoke-test" { options.smoke = true; continue }
        switch arg {
        case "--map", "--screenshot", "--capture-sequence", "--demand-scale", "--warmup":
            if i+1 >= len(args) || args[i+1] == "" || strings.has_prefix(args[i+1], "--") {
                fmt.eprintf("%s requires a value\n", arg)
                return options, false
            }
            i += 1
            value := args[i]
            switch arg {
            case "--map": options.map_path = value
            case "--screenshot": options.screenshot = value
            case "--capture-sequence": options.sequence_dir = value
            case "--demand-scale", "--warmup":
                number, valid := strconv.parse_f64(value)
                if !valid || math.is_nan(number) || math.is_inf(number) {
                    fmt.eprintf("%s requires a finite number\n", arg)
                    return options, false
                }
                if arg == "--demand-scale" {
                    if number <= 0 || number > 20 || f32(number) <= 0 {
                        fmt.eprintln("--demand-scale must be >0 and <=20")
                        return options, false
                    }
                    options.demand_scale = f32(number)
                } else {
                    if number < 0 || number > 3600 {
                        fmt.eprintln("--warmup must be 0..3600 seconds (rounded to 50ms ticks)")
                        return options, false
                    }
                    options.warmup = number
                }
            }
        case:
            fmt.eprintf("Unknown option: %s (see --help)\n", arg)
            return options, false
        }
    }
    // Sequence capture is always a fixed-clock smoke run; no second flag needed.
    if options.sequence_dir != "" { options.smoke = true }
    return options, true
}

// Check before map loading/warmup/window creation. A real exclusive write probe
// tests directory access, not just permission bits (ACLs and read-only mounts).
sequence_directory_ready :: proc(dir: string) -> bool {
    if !os.is_directory(dir) { return false }
    file, err := os.create_temp_file(dir, ".traffic-capture-*")
    if err != nil { return false }
    path := strings.clone(os.name(file))
    defer delete(path)
    count, write_error := os.write(file, []byte{0})
    close_error := os.close(file)
    remove_error := os.remove(path)
    return count == 1 && write_error == nil && close_error == nil && remove_error == nil
}

capture_due :: proc(frame: int) -> bool {
    return frame > 0 && frame <= SEQUENCE_FRAMES && frame%CAPTURE_EVERY == 0
}

// 60 presentation frames/s and 20 simulation ticks/s share an integer clock.
// Normal smoke and sequence prefixes must agree; no wall-clock or RNG sampling.
advance_capture_frame :: proc(world: ^sim.World, frame: int) {
    if frame%3 == 0 { sim.step(world, f32(TICK)) }
}

export_screenshot :: proc(path: string) -> bool {
    image := rl.LoadImageFromScreen()
    defer rl.UnloadImage(image)
    ok := rl.ExportImage(image, strings.clone_to_cstring(path, context.temp_allocator))
    fmt.printf("Screenshot: %s success=%v\n", path, ok)
    if !ok { fmt.eprintf("Capture failed: %s\n", path) }
    return ok
}
