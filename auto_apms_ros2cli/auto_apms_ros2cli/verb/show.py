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

from auto_apms_behavior_tree_core import TreeResource, get_all_behavior_tree_resources
from auto_apms_ros2cli.verb import VerbExtension
from ..api import PrefixFilteredChoicesCompleter


class ShowVerb(VerbExtension):
    """Show the content of a behavior tree resource."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the show verb."""
        tree_id_arg = parser.add_argument(
            "tree_id",
            type=str,
            help="Tree identifier in format: <package>::<file_stem>::<tree_name>",
        )
        trees = [str(tree.identity) for tree in get_all_behavior_tree_resources()]
        tree_id_arg.completer = PrefixFilteredChoicesCompleter(trees)

    def main(self, *, args):
        """Main function for the show verb."""
        tree = TreeResource(args.tree_id)
        print(tree.content)
        return 0
