#!/usr/bin/env python3
# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Benchmarks RobCraft: memory, CPU, loop rate, sim progress, and ROS 2 topic rates."""

from __future__ import annotations

import argparse
import importlib
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

PERF_START_RE = re.compile(r"PERF_START (\d+\.\d+) (\d+\.\d+)")
PERF_RE = re.compile(r"PERF (\d+\.\d+) (\d+\.\d+) (\d+(?:\.\d+)?)")


def parse_perf_start_line(line: str) -> Optional[Tuple[float, float]]:
    """Parses a PERF_START line into (wall_s, sim_s), or None if it does not match."""
    match = PERF_START_RE.search(line)
    if not match:
        return None
    return (float(match.group(1)), float(match.group(2)))


def parse_perf_line(line: str) -> Optional[Tuple[float, float, float]]:
    """Parses a PERF line into (wall_s, sim_s, loop_hz), or None if it does not match."""
    match = PERF_RE.search(line)
    if not match:
        return None
    return (float(match.group(1)), float(match.group(2)), float(match.group(3)))


def parse_proc_status(text: str) -> Dict[str, int]:
    """Extracts VmRSS and VmHWM (kB) from a /proc/<pid>/status dump."""
    fields = {}
    for line in text.splitlines():
        if line.startswith("VmRSS:"):
            fields["rss_kb"] = int(line.split()[1])
        elif line.startswith("VmHWM:"):
            fields["hwm_kb"] = int(line.split()[1])
    return fields


def parse_proc_stat(text: str) -> Dict[str, int]:
    """Extracts utime/stime (clock ticks) from a /proc/<pid>/stat dump."""
    end = text.rfind(")")
    fields = text[end + 2 :].split()
    return {"utime": int(fields[11]), "stime": int(fields[12])}


def compute_cpu_percent(
    utime1: int,
    stime1: int,
    utime2: int,
    stime2: int,
    wall_interval: float,
    clock_ticks: float = 100.0,
) -> Optional[float]:
    """CPU percent of one core between two /proc stat samples."""
    if wall_interval <= 0.0:
        return None
    ticks = (utime2 - utime1) + (stime2 - stime1)
    return ticks / clock_ticks / wall_interval * 100.0


def aggregate(values: List[float]) -> Optional[Dict[str, float]]:
    """Mean/stddev/min/max of a value list, or None when empty."""
    if not values:
        return None
    mean = statistics.mean(values)
    if len(values) == 1:
        stddev = 0.0
    else:
        stddev = statistics.stdev(values)
    return {"mean": mean, "stddev": stddev, "min": min(values), "max": max(values)}


def launch_sim(
    binary: str,
    world: Optional[str],
    headless: bool,
    texture_size: Optional[int],
    perf_stats: bool = True,
) -> subprocess.Popen:
    """Spawns the simulator with the requested options; stdout+stderr merged."""
    cmd = [str(binary)]
    if world:
        cmd.append(world)
    if texture_size is not None:
        cmd.extend(["--texture-size", str(texture_size)])
    if headless:
        cmd.append("--headless")
    if perf_stats:
        cmd.append("--perf-stats")
    return subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )


def terminate_process(proc: subprocess.Popen, timeout: float = 5.0) -> Optional[int]:
    """SIGINT, wait up to timeout, then SIGKILL; returns the exit code."""
    if proc.poll() is not None:
        return proc.returncode
    proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.wait()


def read_stdout(
    proc: subprocess.Popen, stop: threading.Event, out: List[Tuple[float, str]]
) -> None:
    """Reads sim stdout lines into out as (receive_time, line); sets stop on EOF."""
    for line in proc.stdout:
        out.append((time.monotonic(), line))
    stop.set()


def read_proc_samples(
    pid: int,
    interval: float,
    stop: threading.Event,
    out: List[Dict[str, float]],
) -> None:
    """Periodically appends {time, rss_kb, hwm_kb, utime, stime} samples."""
    while not stop.wait(interval):
        try:
            status = Path(f"/proc/{pid}/status").read_text()
            stat = Path(f"/proc/{pid}/stat").read_text()
        except (FileNotFoundError, ProcessLookupError):
            continue
        sample = {"time": time.monotonic()}
        try:
            sample.update(parse_proc_status(status))
            sample.update(parse_proc_stat(stat))
        except (ValueError, IndexError):
            continue
        if not {"rss_kb", "hwm_kb", "utime", "stime"} <= sample.keys():
            continue
        out.append(sample)


def analyze_output(
    lines: List[Tuple[float, str]], t0: float
) -> Tuple[Optional[float], List[float], Optional[float]]:
    """Returns (startup_s, loop_rates, sim_progress) from collected stdout lines."""
    startup_s = None
    loop_rates = []
    perf_entries = []
    for recv, line in lines:
        if startup_s is None:
            match = parse_perf_start_line(line)
            if match is not None:
                startup_s = recv - t0
        match = parse_perf_line(line)
        if match is not None:
            loop_rates.append(match[2])
            perf_entries.append((match[0], match[1]))
    sim_progress = None
    if len(perf_entries) >= 2:
        wall_span = perf_entries[-1][0] - perf_entries[0][0]
        if wall_span > 0.0:
            sim_progress = (perf_entries[-1][1] - perf_entries[0][1]) / wall_span
    return startup_s, loop_rates, sim_progress


def cpu_series(samples: List[Dict[str, float]]) -> List[float]:
    """Per-interval CPU percentages from consecutive /proc samples."""
    series = []
    for prev, cur in zip(samples, samples[1:]):
        interval = cur["time"] - prev["time"]
        pct = compute_cpu_percent(
            prev["utime"], prev["stime"], cur["utime"], cur["stime"], interval
        )
        if pct is not None:
            series.append(pct)
    return series


def topic_rates(
    counts: Dict[str, int],
    first: Dict[str, float],
    last: Dict[str, float],
) -> Dict[str, float]:
    """Messages per second per topic; topics with no span or zero count are omitted."""
    rates = {}
    for topic in counts:
        if (
            topic in first
            and topic in last
            and last[topic] > first[topic]
            and counts[topic] > 0
        ):
            rates[topic] = counts[topic] / (last[topic] - first[topic])
    return rates


class TopicMonitor:
    """Counts messages on /clock and /robot_*/ topics via an rclpy node."""

    _TYPE_MAP = {
        "rosgraph_msgs/msg/Clock": ("rosgraph_msgs.msg", "Clock"),
        "geometry_msgs/msg/Twist": ("geometry_msgs.msg", "Twist"),
        "nav_msgs/msg/Odometry": ("nav_msgs.msg", "Odometry"),
        "sensor_msgs/msg/LaserScan": ("sensor_msgs.msg", "LaserScan"),
        "sensor_msgs/msg/Imu": ("sensor_msgs.msg", "Imu"),
        "sensor_msgs/msg/NavSatFix": ("sensor_msgs.msg", "NavSatFix"),
        "sensor_msgs/msg/MagneticField": ("sensor_msgs.msg", "MagneticField"),
        "sensor_msgs/msg/Image": ("sensor_msgs.msg", "Image"),
        "sensor_msgs/msg/CameraInfo": ("sensor_msgs.msg", "CameraInfo"),
        "sensor_msgs/msg/PointCloud2": ("sensor_msgs.msg", "PointCloud2"),
    }

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled
        self.counts: Dict[str, int] = {}
        self.first: Dict[str, float] = {}
        self.last: Dict[str, float] = {}
        self.sim_first: Dict[str, float] = {}
        self.sim_last: Dict[str, float] = {}
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._executor = None
        self._node = None
        self._subscribed: Set[str] = set()

    def start(self) -> None:
        """Creates the rclpy node and subscriptions; no-op when disabled."""
        if not self.enabled:
            return
        self.counts = {}
        self.first = {}
        self.last = {}
        self.sim_first = {}
        self.sim_last = {}
        try:
            import rclpy
        except ImportError as exc:
            print(
                f"warning: rclpy unavailable ({exc}); skipping ROS 2 topic metrics",
                file=sys.stderr,
            )
            self.enabled = False
            return
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("robcraft_benchmark")
        self._subscribed = set()
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin_and_discover, daemon=True)
        self._thread.start()

    def _spin_and_discover(self) -> None:
        """Spins the executor, retrying topic discovery for up to 5 s."""
        deadline = time.monotonic() + 5.0
        while (
            self._executor is not None
            and self._executor._context.ok()
            and not self._executor._is_shutdown
        ):
            if time.monotonic() < deadline:
                self._discover_and_subscribe()
            self._executor.spin_once(timeout_sec=0.25)

    def _discover_and_subscribe(self) -> None:
        """Subscribes to /clock and /robot_*/ topics currently in the graph."""
        topics = self._node.get_topic_names_and_types()
        items = topics.items() if isinstance(topics, dict) else topics
        for name, types in items:
            if name in self._subscribed:
                continue
            if name == "/clock" or re.match(r"^/robot_[^/]+/", name):
                self._subscribe(name, types[0])

    def _subscribe(self, name: str, type_name: str) -> None:
        entry = self._TYPE_MAP.get(type_name)
        if entry is None:
            print(
                f"warning: skipping {name} with unknown type {type_name}",
                file=sys.stderr,
            )
            return
        module = importlib.import_module(entry[0])
        msg_cls = getattr(module, entry[1])
        self._node.create_subscription(msg_cls, name, self._make_callback(name), 10)
        self._subscribed.add(name)

    def _make_callback(self, topic: str) -> Callable[[object], None]:
        def callback(msg):
            now = time.monotonic()
            with self._lock:
                self.counts[topic] = self.counts.get(topic, 0) + 1
                self.first.setdefault(topic, now)
                self.last[topic] = now
                if topic == "/clock":
                    sim = msg.clock.sec + msg.clock.nanosec * 1e-9
                    self.sim_first.setdefault(topic, sim)
                    self.sim_last[topic] = sim

        return callback

    def stop(self) -> None:
        """Stops the executor, destroys the node, and shuts rclpy down."""
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._executor is not None:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        self._executor = None
        self._thread = None
        self._node = None

    def rates(self) -> Dict[str, float]:
        """Messages per second per subscribed topic."""
        with self._lock:
            return topic_rates(dict(self.counts), dict(self.first), dict(self.last))

    def clock_progress(self) -> Optional[float]:
        """Sim-time span divided by wall-time span from /clock messages."""
        with self._lock:
            if "/clock" not in self.sim_first or "/clock" not in self.last:
                return None
            wall_span = self.last["/clock"] - self.first["/clock"]
            if wall_span <= 0.0:
                return None
            if self.sim_last["/clock"] <= self.sim_first["/clock"]:
                return None
            return (self.sim_last["/clock"] - self.sim_first["/clock"]) / wall_span


def run_sample(binary, world, headless, texture_size, duration, use_ros):
    """Runs the sim once for `duration` seconds and returns the sample metrics."""
    t0 = time.monotonic()
    proc = launch_sim(binary, world, headless, texture_size)
    monitor = TopicMonitor(enabled=use_ros)
    monitor.start()
    lines = []
    samples = []
    stop = threading.Event()
    threads = [
        threading.Thread(target=read_stdout, args=(proc, stop, lines), daemon=True),
        threading.Thread(
            target=read_proc_samples, args=(proc.pid, 0.5, stop, samples), daemon=True
        ),
    ]
    for thread in threads:
        thread.start()
    deadline = t0 + duration
    try:
        while time.monotonic() < deadline and proc.poll() is None:
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop.set()
        monitor.stop()
        terminate_process(proc)
        raise
    stop.set()
    exit_code = terminate_process(proc)
    for thread in threads:
        thread.join(timeout=2.0)
    rates = monitor.rates()
    clock_progress = monitor.clock_progress()
    monitor.stop()
    startup_s, loop_rates, perf_progress = analyze_output(lines, t0)
    result = {
        "startup_s": startup_s,
        "loop_rate_hz": aggregate(loop_rates),
        "sim_progress_perf": perf_progress,
        "sim_progress_clock": clock_progress,
        "rss_kb": aggregate([s["rss_kb"] for s in samples]),
        "peak_rss_kb": max((s["hwm_kb"] for s in samples), default=None),
        "cpu_percent": aggregate(cpu_series(samples)),
        "topic_rates_hz": rates,
        "exit_code": exit_code,
    }
    if exit_code != 0:
        result["output_tail"] = [line for _, line in lines[-20:]]
    return result


def summarize_samples(samples):
    """Aggregates the per-sample metrics across samples."""
    summary = {}
    for key in ("startup_s", "sim_progress_perf", "sim_progress_clock", "peak_rss_kb"):
        summary[key] = aggregate([s[key] for s in samples if s[key] is not None])
    for key in ("loop_rate_hz", "rss_kb", "cpu_percent"):
        summary[key] = {
            sub: aggregate([s[key][sub] for s in samples if s[key] is not None])
            for sub in ("mean", "stddev", "min", "max")
        }
    topic_names = sorted({t for s in samples for t in s["topic_rates_hz"]})
    summary["topic_rates_hz"] = {
        topic: aggregate(
            [s["topic_rates_hz"][topic] for s in samples if topic in s["topic_rates_hz"]]
        )
        for topic in topic_names
    }
    return summary


def fmt(agg, unit=""):
    """Formats an aggregate dict as 'mean ± stddev unit', or 'n/a'."""
    if agg is None or agg["mean"] is None:
        return "n/a"
    sub = agg["mean"]
    if isinstance(sub, dict):
        return f"{sub['mean']:.2f} ± {sub['stddev']:.2f} {unit}".strip()
    return f"{agg['mean']:.2f} ± {agg['stddev']:.2f} {unit}".strip()


def print_table(results):
    """Prints a human-readable summary table to the terminal."""
    for mode, mode_results in results["modes"].items():
        summary = mode_results["summary"]
        print(f"\n=== {mode} ===")
        print(f"  startup_s:      {fmt(summary['startup_s'])}")
        print(f"  loop_rate_hz:   {fmt(summary['loop_rate_hz'])}")
        print(
            "  sim_progress:   "
            f"{fmt(summary['sim_progress_perf'])} (perf) / "
            f"{fmt(summary['sim_progress_clock'])} (clock)"
        )
        print(
            f"  rss_kb:         {fmt(summary['rss_kb'])}  peak: {fmt(summary['peak_rss_kb'])}"
        )
        print(f"  cpu_percent:    {fmt(summary['cpu_percent'])}")
        print("  topic_rates_hz:")
        for topic, agg in sorted(summary["topic_rates_hz"].items()):
            print(f"    {topic}: {fmt(agg)}")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Benchmark RobCraft performance and memory usage"
    )
    parser.add_argument(
        "--world",
        default="robcraft/worlds/maze.world",
        help=".world file to load (default: robcraft/worlds/maze.world)",
    )
    parser.add_argument(
        "--mode", choices=["headless", "windowed", "both"], default="headless"
    )
    parser.add_argument(
        "--duration", type=float, default=30.0, help="seconds per sample (default: 30)"
    )
    parser.add_argument(
        "--texture-size", type=int, default=None, help="texture size: 256, 512, or 1024"
    )
    parser.add_argument(
        "--samples", type=int, default=1, help="repetitions per mode (default: 1)"
    )
    parser.add_argument(
        "--binary",
        default="build/robcraft/robcraft",
        help="path to the simulator binary (default: build/robcraft/robcraft)",
    )
    parser.add_argument(
        "--out",
        default="results",
        help="output directory for JSON results (default: results)",
    )
    parser.add_argument(
        "--no-ros", action="store_true", help="skip ROS 2 topic monitoring"
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    binary = Path(args.binary)
    if not binary.is_file() or not os.access(binary, os.X_OK):
        sys.exit(f"error: binary not found or not executable: {binary}")
    if args.world and not Path(args.world).is_file():
        sys.exit(f"error: world file not found: {args.world}")
    modes = ["headless", "windowed"] if args.mode == "both" else [args.mode]
    results = {
        "world": args.world,
        "duration_s": args.duration,
        "texture_size": args.texture_size,
        "samples": args.samples,
        "modes": {},
    }
    try:
        for mode in modes:
            sample_results = [
                run_sample(
                    binary,
                    args.world,
                    mode == "headless",
                    args.texture_size,
                    args.duration,
                    not args.no_ros,
                )
                for _ in range(args.samples)
            ]
            results["modes"][mode] = {
                "samples": sample_results,
                "summary": summarize_samples(sample_results),
            }
    except KeyboardInterrupt:
        print("\nInterrupted; saving partial results", file=sys.stderr)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    world_stem = Path(args.world).stem if args.world else "empty"
    out_path = (
        out_dir / f"bench-{world_stem}-{args.mode}-{time.strftime('%Y%m%d-%H%M%S')}.json"
    )
    out_path.write_text(json.dumps(results, indent=2) + "\n")
    print(f"\nResults written to {out_path}")
    print_table(results)


if __name__ == "__main__":
    main()
