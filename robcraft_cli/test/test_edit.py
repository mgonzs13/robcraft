import subprocess

from robcraft_cli.verb.edit import run_editor


def test_run_editor_command(monkeypatch):
    captured = {}

    def fake_run(command, check=False):
        captured["command"] = command
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_editor() == 0
    assert captured["command"] == ["ros2", "run", "robcraft", "robcraft-editor"]


def test_run_editor_keyboard_interrupt(monkeypatch):
    def fake_run(command, check=False):
        raise KeyboardInterrupt

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_editor() == 130


def test_run_editor_file_not_found(monkeypatch, capsys):
    def fake_run(command, check=False):
        raise FileNotFoundError("ros2")

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert run_editor() == 1
    assert "Failed to execute command" in capsys.readouterr().out
