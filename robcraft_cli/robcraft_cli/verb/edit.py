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


def run_editor() -> int:
    command = ["ros2", "run", "robcraft", "robcraft-editor"]
    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        return 130
    except FileNotFoundError as exc:
        print(f"Failed to execute command: {exc}")
        return 1


def add_edit_verb(subparsers):
    parser = subparsers.add_parser(
        "edit",
        help="Open the RobCraft world editor",
        description="Open the RobCraft WC3-style world editor",
    )
    parser.set_defaults(main=_main_edit)


def _main_edit(args):
    return run_editor()
