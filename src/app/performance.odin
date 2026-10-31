package app

import "core:fmt"
import "core:math"
import "core:slice"

Frame_Sample :: struct {
    frame_ms, sim_ms, draw_ms, present_ms: f64,
    ticks: int,
}

Frame_Stats :: struct {
    seconds, fps, mean_ms, p50_ms, p95_ms, p99_ms, max_ms: f64,
    sim_ms, draw_ms, present_ms, sim_max_ms: f64,
    ticks, without_tick, over25, over50: int,
}

frame_stats :: proc(samples: []Frame_Sample) -> (s: Frame_Stats) {
    if len(samples) == 0 { return }
    times := make([]f64, len(samples))
    defer delete(times)
    for frame, i in samples {
        times[i] = frame.frame_ms
        s.seconds += frame.frame_ms/1000
        s.sim_ms += frame.sim_ms
        s.draw_ms += frame.draw_ms
        s.present_ms += frame.present_ms
        s.sim_max_ms = math.max(s.sim_max_ms, frame.sim_ms)
        s.ticks += frame.ticks
        if frame.ticks == 0 { s.without_tick += 1 }
        if frame.frame_ms > 25 { s.over25 += 1 }
        if frame.frame_ms > 50 { s.over50 += 1 }
    }
    n := f64(len(samples))
    s.fps = n/s.seconds // Frames / elapsed time, not the mean of instantaneous FPS.
    s.mean_ms = s.seconds*1000/n
    s.sim_ms /= n
    s.draw_ms /= n
    s.present_ms /= n
    slice.sort(times)
    // Nearest-rank percentiles; sort a copy so samples retain their original order.
    s.p50_ms = times[int(math.ceil(n*0.50))-1]
    s.p95_ms = times[int(math.ceil(n*0.95))-1]
    s.p99_ms = times[int(math.ceil(n*0.99))-1]
    s.max_ms = times[len(times)-1]
    return
}

print_frame_report :: proc(samples: []Frame_Sample, uncapped: bool) {
    if len(samples) == 0 { fmt.println("Benchmark: no measured frames"); return }
    s := frame_stats(samples)
    fmt.printf("Benchmark: %d frames / %.3fs | limiter=%s | 2s render warmup excluded\n",
        len(samples), s.seconds, "off" if uncapped else "60 FPS")
    fmt.printf("FPS %.2f | frame ms: mean %.2f, p50 %.2f, p95 %.2f, p99 %.2f, max %.2f\n",
        s.fps, s.mean_ms, s.p50_ms, s.p95_ms, s.p99_ms, s.max_ms)
    fmt.printf("Slow frames: >25ms %d (%.2f%%), >50ms %d (%.2f%%)\n",
        s.over25, 100*f64(s.over25)/f64(len(samples)), s.over50, 100*f64(s.over50)/f64(len(samples)))
    fmt.printf("Mean ms/frame: simulation %.3f (max %.3f), draw submission %.3f, end/present %.3f\n",
        s.sim_ms, s.sim_max_ms, s.draw_ms, s.present_ms)
    fmt.printf("Simulation: %d ticks (%.2f Hz), %d/%d frames without a tick; poses interpolated at render cadence (one tick delay)\n",
        s.ticks, f64(s.ticks)/s.seconds, s.without_tick, len(samples))
    fmt.println("End/present includes batch flush, swap, FPS-limiter wait and event polling; not a GPU timer.")
}
