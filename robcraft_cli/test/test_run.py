import subprocess

from robcraft_cli.verb.run import build_run_command, run_simulator


def test_build_run_command_empty():
    assert build_run_command("", None) == ["ros2", "run", "robcraft", "robcraft"]


def test_build_run_command_world():
    assert build_run_command("worlds/maze.world", None) == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "worlds/maze.world",
    ]


def test_build_run_command_texture_size():
    assert build_run_command("", 512) == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "--texture-size",
        "512",
    ]


def test_build_run_command_both():
    assert build_run_command("worlds/maze.world", 512) == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "worlds/maze.world",
        "--texture-size",
        "512",
    ]


def test_run_simulator_returns_child_exit_code(monkeypatch):
    class FakeCompleted:
        returncode = 3

    def fake_run(command, check=False):
        return FakeCompleted()

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_simulator("worlds/maze.world", 256) == 3


def test_run_simulator_keyboard_interrupt(monkeypatch):
    def fake_run(command, check=False):
        raise KeyboardInterrupt

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_simulator() == 130


def test_run_simulator_file_not_found(monkeypatch, capsys):
    def fake_run(command, check=False):
        raise FileNotFoundError("ros2")

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_simulator() == 1
    assert "Failed to execute command" in capsys.readouterr().out


def test_build_run_command_headless():
    assert build_run_command("", None, True) == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "--headless",
    ]


def test_build_run_command_all():
    assert build_run_command("worlds/maze.world", 512, True) == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "worlds/maze.world",
        "--texture-size",
        "512",
        "--headless",
    ]


def test_run_simulator_headless_command(monkeypatch):
    captured = {}

    def fake_run(command, check=False):
        captured["command"] = command
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_simulator("worlds/maze.world", None, True) == 0
    assert captured["command"] == [
        "ros2",
        "run",
        "robcraft",
        "robcraft",
        "worlds/maze.world",
        "--headless",
    ]
