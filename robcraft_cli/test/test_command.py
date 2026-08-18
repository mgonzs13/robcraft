import argparse

import pytest

from robcraft_cli.command.robcraft import RobcraftCommand


def test_add_arguments_registers_verbs():
    command = RobcraftCommand()
    parser = argparse.ArgumentParser(prog="robcraft")
    command.add_arguments(parser, "robcraft")

    args = parser.parse_args(["run", "worlds/maze.world", "--texture-size", "512"])
    assert args.verb == "run"
    assert args.world == "worlds/maze.world"
    assert args.texture_size == 512
    assert args.headless is False

    args = parser.parse_args(["run", "--headless"])
    assert args.verb == "run"
    assert args.headless is True

    args = parser.parse_args(["edit"])
    assert args.verb == "edit"


def test_add_arguments_requires_verb():
    command = RobcraftCommand()
    parser = argparse.ArgumentParser(prog="robcraft")
    command.add_arguments(parser, "robcraft")
    with pytest.raises(SystemExit):
        parser.parse_args([])


def test_main_dispatches_verb_main(monkeypatch):
    command = RobcraftCommand()

    class Args:
        pass

    args = Args()
    calls = []

    def fake_main(value):
        calls.append(value)
        return 7

    args.main = fake_main
    fake_exit = {"code": None}

    def fake_os_exit(code):
        fake_exit["code"] = code

    monkeypatch.setattr("robcraft_cli.command.robcraft.os._exit", fake_os_exit)
    command.main(parser=None, args=args)

    assert calls == [args]
    assert fake_exit["code"] == 7


def test_main_dispatches_verb_main_exit_zero_when_result_none(monkeypatch):
    command = RobcraftCommand()

    class Args:
        pass

    args = Args()

    def fake_main(value):
        return None

    args.main = fake_main
    fake_exit = {"code": None}

    def fake_os_exit(code):
        fake_exit["code"] = code

    monkeypatch.setattr("robcraft_cli.command.robcraft.os._exit", fake_os_exit)
    command.main(parser=None, args=args)

    assert fake_exit["code"] == 0
