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

import os
import sys

from ros2cli.command import CommandExtension

from robcraft_cli.verb.edit import add_edit_verb
from robcraft_cli.verb.run import add_run_verb


class RobcraftCommand(CommandExtension):
    """RobCraft command line tools."""

    def add_arguments(self, parser, cli_name):
        subparsers = parser.add_subparsers(dest="verb", metavar="verb")
        subparsers.required = True

        add_run_verb(subparsers)
        add_edit_verb(subparsers)

        self._parser = parser

    def main(self, *, parser, args):
        verb_main = getattr(args, "main", None)
        if verb_main is None:
            self._parser.print_help()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)

        result = verb_main(args)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(result if result is not None else 0)
