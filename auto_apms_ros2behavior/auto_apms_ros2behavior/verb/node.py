# Copyright 2025 Robin Müller
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

from ros2cli.command import add_subparsers_on_demand
from ..verb import VerbExtension


class TestVerb(VerbExtension):
    """List all available behavior resources."""

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        print("TEST")
        return 0


class NodeVerb(VerbExtension):
    """Subcommand for everything related to behavior tree nodes."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(parser, cli_name, "_node_verb", "auto_apms_ros2behavior.verb.node", required=False)

    def main(self, *, args):
        if not hasattr(args, "_node_verb"):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_node_verb")

        # call the verb's main method
        return extension.main(args=args)
