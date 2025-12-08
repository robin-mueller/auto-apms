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

"""AutoAPMS behavior management command for ros2cli."""

from ros2cli.command import CommandExtension, add_subparsers_on_demand
from auto_apms_behavior_tree_core.resources import get_behavior_resource_identities
from ..api import print_grouped_behavior_identities


class BehaviorCommand(CommandExtension):
    """Inspect and deploy behaviors created with AutoAPMS."""

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        parser.add_argument(
            "-l",
            "--list",
            action="store_true",
            help="List all available behavior resources",
        )
        add_subparsers_on_demand(parser, cli_name, "_verb", "auto_apms_ros2behavior.verb", required=False)

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            if args.list:
                print_grouped_behavior_identities(get_behavior_resource_identities(), group_by="category")
                return 0

            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb")

        # call the verb's main method
        return extension.main(args=args)
