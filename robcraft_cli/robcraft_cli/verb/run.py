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

from __future__ import annotations

import subprocess
from typing import List, Optional


def build_run_command(
    world: str, texture_size: Optional[int], headless: bool = False
) -> List[str]:
    command = ["ros2", "run", "robcraft", "robcraft"]
    if world:
        command.append(world)
    if texture_size is not None:
        command.extend(["--texture-size", str(texture_size)])
    if headless:
        command.append("--headless")
    return command


def run_simulator(
    world: str = "", texture_size: Optional[int] = None, headless: bool = False
) -> int:
    command = build_run_command(world, texture_size, headless)
    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


def add_run_verb(subparsers):
    parser = subparsers.add_parser(
        "run",
        help="Run the RobCraft simulator",
        description="Run the RobCraft simulator, optionally with a world file",
    )
    parser.add_argument(
        "world",
        nargs="?",
        default="",
        help="Optional path to a .world file; simulator starts empty without it",
    )
    parser.add_argument(
        "--texture-size",
        type=int,
        metavar="N",
        default=None,
        help="Shared terrain/building texture size: 256, 512, or 1024 (default 256)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without a visible window (still requires a display server)",
    )
    parser.set_defaults(main=_main_run)


def _main_run(args):
    return run_simulator(args.world, args.texture_size, args.headless)
