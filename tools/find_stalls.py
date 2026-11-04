#!/usr/bin/env python3
"""Audit exact stopped drivers through the persistent native JSON editor CLI."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import math


class ZeroRunTracker:
    def __init__(self):
        self.active = {}
        self.finished = []

    def update(self, time, drivers):
        present = {driver["id"]: driver for driver in drivers}
        started = []
        for driver_id, run in list(self.active.items()):
            driver = present.get(driver_id)
            if (driver is None or driver["speed"] != 0.0 or
                    driver["position"] != run["last_position"]):
                self.finished.append(run)
                del self.active[driver_id]
        for driver_id, driver in present.items():
            if driver["speed"] == 0.0:
                if driver_id not in self.active:
                    run = {
                        "id": driver_id, "begin": time, "end": time,
                        "last_position": driver["position"], "samples": 1,
                        "clear_since": None, "max_clear_seconds": 0.0,
                    }
                    self.active[driver_id] = run
                    started.append(run)
                else:
                    run = self.active[driver_id]
                    run["end"] = time
                    run["samples"] += 1
                if driver.get("steering_available", False):
                    if run["clear_since"] is None:
                        run["clear_since"] = time
                    run["max_clear_seconds"] = max(run["max_clear_seconds"], time - run["clear_since"])
                else:
                    run["clear_since"] = None
            # Nonzero drivers were removed above; they cannot bridge a run.
        return started

    def finish(self, stopped_seconds):
        self.finished.extend(self.active.values())
        self.active.clear()
        for run in self.finished:
            run["duration"] = run["end"] - run["begin"]
            run["transient"] = run["duration"] < stopped_seconds
        return self.finished


class Editor:
    def __init__(self, binary, map_path):
        self.process = subprocess.Popen(
            [binary, "--session", map_path], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, text=True,
            bufsize=1,
        )
        self.closed = False
        ready = self._read()
        if ready.get("op") != "ready" or not ready.get("ok"):
            self.close()
            raise RuntimeError(f"editor failed to initialize: {ready}")

    def _read(self):
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(f"editor closed stdout (exit={self.process.poll()}); see stderr")
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise RuntimeError(f"invalid editor JSON stdout: {line.rstrip()!r}") from error
        if not isinstance(value, dict) or not isinstance(value.get("op"), str) or not isinstance(value.get("ok"), bool):
            raise RuntimeError(f"invalid editor response: {value!r}")
        return value

    def command(self, command):
        if self.closed:
            raise RuntimeError("editor session is closed")
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        response = self._read()
        if response["op"] != command.split()[0]:
            raise RuntimeError(f"response mismatch for {command!r}: {response!r}")
        if not response["ok"]:
            raise RuntimeError(f"editor rejected {command!r}: {response}")
        return response

    def close(self):
        if self.closed:
            return
        try:
            if self.process.poll() is None:
                self.command("quit")
        finally:
            self.process.stdin.close()
            status = self.process.wait()
            self.closed = True
        if status != 0:
            raise RuntimeError(f"editor exited {status}; see stderr")


def inspect_run(editor, driver_id, snapshot, run):
    editor.command(f"select {driver_id}")
    editor.command(f"save {snapshot}")
    inspector = editor.command(f"inspect {driver_id}")["details"]
    probes = {}
    for distance in (0.75, 1.5, 3.0):
        probes[str(distance)] = editor.command(f"probe {distance}")["details"]
    available = any(probe["available"] for probe in probes.values())
    run.update({
        "snapshot": str(snapshot), "inspector": inspector, "probes": probes,
        "classification": "local_space_suspect" if available else "no_local_space_found",
    })


def audit(args):
    if (not math.isfinite(args.until) or not math.isfinite(args.stopped_seconds) or
            args.until < 0 or args.until > 3600 or args.step_ticks < 1 or args.step_ticks > 20000 or args.stopped_seconds <= 0):
        raise ValueError("until must be 0..3600, step-ticks 1..20000, and stopped-seconds positive")
    until_ticks = round(args.until * 20)
    if abs(until_ticks / 20 - args.until) > 1e-9:
        raise ValueError("--until must align to 50ms simulation ticks")
    if not Path(args.binary).is_file() or not os.access(args.binary, os.X_OK):
        raise ValueError(f"binary is not an executable file: {args.binary}")
    if not Path(args.map).is_file():
        raise ValueError(f"map does not exist: {args.map}")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    tracker = ZeroRunTracker()
    editor = Editor(args.binary, args.map)
    try:
        editor.command("seek 0")
        tick = 0
        while tick <= until_ticks:
            response = editor.command("list stopped")
            drivers = response.get("details")
            if not isinstance(drivers, list):
                raise RuntimeError("list response has no driver array")
            when = tick / 20
            tracker.update(when, drivers)
            for run in tracker.active.values():
                if ("snapshot" not in run and run["clear_since"] is not None and
                        when - run["clear_since"] >= args.stopped_seconds):
                    snapshot = output / f"t{when:.2f}-driver{run['id']}.cbor"
                    inspect_run(editor, run["id"], snapshot, run)
                    run["snapshot_time"] = when
                    print(f"SUSPECT id={run['id']} stationary_since={run['begin']:.2f}s observed={when:.2f}s", file=sys.stderr)
            if tick % 600 == 0:
                print(f"SCAN t={when:.2f}s completed={response['completed']} active={response['active']}", file=sys.stderr)
            if tick == until_ticks:
                break
            advance = min(args.step_ticks, until_ticks - tick)
            editor.command(f"step {advance}")
            tick += advance
        terminal_snapshot = None
        if drivers:
            editor.command(f"select {min(driver['id'] for driver in drivers)}")
            terminal_snapshot = str(output / "final.cbor")
            editor.command(f"save {terminal_snapshot}")
    finally:
        editor.close()
    zero_runs = tracker.finish(args.stopped_seconds)
    for run in zero_runs:
        run.pop("last_position", None)
        run.pop("samples", None)
    report = {
        "until_seconds": args.until, "step_ticks": args.step_ticks,
        "sample_seconds": args.step_ticks / 20,
        "stopped_seconds": args.stopped_seconds, "zero_runs": zero_runs,
        "terminal_snapshot": terminal_snapshot,
        "completed": response["completed"], "active": response["active"],
    }
    destination = output / "temporal-report.json"
    destination.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return destination, len(zero_runs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default="build/replay-case")
    parser.add_argument("--map", default="build/maps/intersection.csv")
    parser.add_argument("--until", type=float, default=600)
    parser.add_argument("--step-ticks", type=int, default=5)
    parser.add_argument("--stopped-seconds", type=float, default=2)
    parser.add_argument("--output", default="build/stall-audit/temporal")
    args = parser.parse_args()
    try:
        path, count = audit(args)
    except (OSError, RuntimeError, ValueError, BrokenPipeError) as error:
        print(f"find_stalls: {error}", file=sys.stderr)
        return 2
    print(f"Wrote {path}; recorded {count} zero-speed runs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
