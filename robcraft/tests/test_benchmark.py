import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from benchmark import (
    aggregate,
    analyze_output,
    compute_cpu_percent,
    cpu_series,
    parse_perf_line,
    parse_perf_start_line,
    parse_proc_stat,
    parse_proc_status,
    topic_rates,
)


def test_parse_perf_start_line():
    assert parse_perf_start_line("PERF_START 1.234 0.500") == (1.234, 0.5)


def test_parse_perf_start_line_ignores_other_lines():
    assert parse_perf_start_line("PERF 1.234 0.500 123.4") is None
    assert parse_perf_start_line("Loaded world: worlds/maze.world") is None


def test_parse_perf_line():
    assert parse_perf_line("PERF 2.500 1.000 240.5") == (2.5, 1.0, 240.5)


def test_parse_perf_line_ignores_other_lines():
    assert parse_perf_line("PERF_START 1.234 0.500") is None
    assert parse_perf_line("info: PERF 2.5 1.0 240.5") == (2.5, 1.0, 240.5)


def test_parse_proc_status():
    text = "Name:\trobcraft\nVmRSS:\t  123456 kB\nVmHWM:\t  130000 kB\n"
    assert parse_proc_status(text) == {"rss_kb": 123456, "hwm_kb": 130000}


def test_parse_proc_stat():
    # The comm field contains parentheses; utime/stime are fields 14/15.
    text = "12345 (robcraft) S 1 2 3 4 5 6 7 8 9 10 140 150 16"
    assert parse_proc_stat(text) == {"utime": 140, "stime": 150}


def test_compute_cpu_percent():
    # 50 ticks over a 1.0 s wall interval at 100 ticks/s = 50% of one core.
    assert compute_cpu_percent(0, 0, 30, 20, 1.0) == pytest.approx(50.0)


def test_compute_cpu_percent_zero_interval():
    assert compute_cpu_percent(0, 0, 10, 10, 0.0) is None


def test_aggregate():
    result = aggregate([1.0, 2.0, 3.0])
    assert result["mean"] == pytest.approx(2.0)
    assert result["stddev"] == pytest.approx(1.0)
    assert result["min"] == 1.0
    assert result["max"] == 3.0


def test_aggregate_single_value():
    result = aggregate([5.0])
    assert result["mean"] == 5.0
    assert result["stddev"] == 0.0


def test_aggregate_empty():
    assert aggregate([]) is None


def test_analyze_output():
    t0 = 100.0
    lines = [
        (100.5, "PERF_START 0.300 0.000"),
        (101.5, "PERF 1.200 1.000 250.0"),
        (102.5, "PERF 2.200 2.000 240.0"),
    ]
    startup_s, loop_rates, sim_progress = analyze_output(lines, t0)
    assert startup_s == pytest.approx(0.5)
    assert loop_rates == pytest.approx([250.0, 240.0])
    assert sim_progress == pytest.approx(1.0)


def test_analyze_output_ignores_other_lines():
    lines = [(100.5, "Loaded world: worlds/maze.world"), (101.0, "LiDAR E3: 360 rays")]
    startup_s, loop_rates, sim_progress = analyze_output(lines, 100.0)
    assert startup_s is None
    assert loop_rates == []
    assert sim_progress is None


def test_analyze_output_single_perf_entry():
    lines = [(101.5, "PERF 1.200 1.000 250.0")]
    startup_s, loop_rates, sim_progress = analyze_output(lines, 100.0)
    assert startup_s is None
    assert loop_rates == [250.0]
    assert sim_progress is None


def test_analyze_output_zero_wall_span():
    lines = [
        (101.5, "PERF 1.200 1.000 250.0"),
        (101.5, "PERF 1.200 1.100 240.0"),
    ]
    startup_s, loop_rates, sim_progress = analyze_output(lines, 100.0)
    assert startup_s is None
    assert loop_rates == [250.0, 240.0]
    assert sim_progress is None


def test_cpu_series():
    samples = [
        {"time": 0.0, "utime": 0, "stime": 0},
        {"time": 1.0, "utime": 30, "stime": 20},
        {"time": 2.0, "utime": 60, "stime": 40},
    ]
    assert cpu_series(samples) == pytest.approx([50.0, 50.0])


def test_topic_rates():
    counts = {"/clock": 100, "/robot_1/scan": 0}
    first = {"/clock": 10.0, "/robot_1/scan": 10.0}
    last = {"/clock": 20.0, "/robot_1/scan": 20.0}
    rates = topic_rates(counts, first, last)
    assert rates["/clock"] == pytest.approx(10.0)
    assert "/robot_1/scan" not in rates
