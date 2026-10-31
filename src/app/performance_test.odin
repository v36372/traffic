package app

import "core:math"
import "core:testing"

@(test)
test_frame_statistics_preserve_hitches_and_weight_fps_by_elapsed_time :: proc(t: ^testing.T) {
    empty := frame_stats(nil)
    testing.expect(t, empty == Frame_Stats{})
    single := frame_stats([]Frame_Sample{{frame_ms = 20}})
    testing.expect(t, single.fps == 50 && single.p50_ms == 20 && single.p99_ms == 20)

    pair := []Frame_Sample{
        {frame_ms = 30, sim_ms = 6, draw_ms = 4, present_ms = 20, ticks = 2},
        {frame_ms = 10, sim_ms = 0, draw_ms = 2, present_ms = 8},
    }
    stats := frame_stats(pair)
    testing.expect(t, math.abs(stats.fps-50) < 0.000001 && stats.mean_ms == 20)
    testing.expect(t, stats.p50_ms == 10 && stats.p95_ms == 30 && stats.p99_ms == 30 && stats.max_ms == 30)
    testing.expect(t, stats.sim_ms == 3 && stats.sim_max_ms == 6 && stats.draw_ms == 3 && stats.present_ms == 14)
    testing.expect(t, stats.ticks == 2 && stats.without_tick == 1 && stats.over25 == 1 && stats.over50 == 0)
    testing.expect(t, pair[0].frame_ms == 30) // Sorting must not mutate measurements.

    frames: [100]Frame_Sample
    for &frame in frames { frame.frame_ms = 10 }
    frames[0].frame_ms = 100
    frames[99].frame_ms = 100
    stats = frame_stats(frames[:])
    testing.expect(t, math.abs(stats.mean_ms-11.8) < 0.000001)
    testing.expect(t, math.abs(stats.fps-100000.0/1180) < 0.000001)
    testing.expect(t, stats.p95_ms == 10 && stats.p99_ms == 100 && stats.max_ms == 100)
    testing.expect(t, stats.over25 == 2 && stats.over50 == 2 && stats.without_tick == 100)
}

@(test)
test_benchmark_options_validate_duration_and_reject_capture_io :: proc(t: ^testing.T) {
    defaults, ok := parse_options(nil)
    testing.expect(t, ok && defaults.benchmark == 0 && !defaults.uncapped)
    live, live_ok := parse_options([]string{"--uncapped"})
    testing.expect(t, live_ok && live.uncapped && live.benchmark == 0 && !live.smoke)
    for value in ([]string{"", "--uncapped", "0", "-1", "0.5", "300.01", "nan", "inf", "-inf", "abc", "1e999"}) {
        _, accepted := parse_options([]string{"--benchmark", value})
        testing.expect(t, !accepted)
    }
    _, missing := parse_options([]string{"--benchmark"})
    testing.expect(t, !missing)
    for duration in ([]string{"1", "15.5", "300"}) {
        options, valid := parse_options([]string{"--benchmark", duration, "--uncapped"})
        testing.expect(t, valid && options.benchmark >= 1 && options.benchmark <= 300 && options.uncapped && !options.smoke)
    }
    for conflict in ([][]string{
        {"--smoke-test", "--benchmark", "1"},
        {"--benchmark", "1", "--capture-sequence", "frames"},
        {"--benchmark", "1", "--screenshot", "frame.png"},
        {"--uncapped", "--smoke-test"},
        {"--capture-sequence", "frames", "--uncapped"},
    }) {
        _, accepted := parse_options(conflict)
        testing.expect(t, !accepted)
    }
}
