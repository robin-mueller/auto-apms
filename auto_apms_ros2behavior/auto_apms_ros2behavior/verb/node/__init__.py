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
from auto_apms_behavior_tree_core.resources import (
    NodeManifest,
    get_node_manifest_resource_identities,
)
from ...api import print_manifest_node_names
from ...verb import VerbExtension


class NodeVerb(VerbExtension):
    """Subcommand for everything related to behavior tree nodes."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        parser.add_argument(
            "-l",
            "--list",
            action="store_true",
            help="List all available behavior tree nodes grouped by node manifest",
        )
        add_subparsers_on_demand(parser, cli_name, "_verb_node", "auto_apms_ros2behavior.verb.node", required=False)

    def main(self, *, args):
        if not hasattr(args, "_verb_node"):
            if args.list:
                for i in get_node_manifest_resource_identities():
                    print(str(i))
                    print_manifest_node_names(NodeManifest.from_resource(i))
                return 0

            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb_node")

        # call the verb's main method
        return extension.main(args=args)
