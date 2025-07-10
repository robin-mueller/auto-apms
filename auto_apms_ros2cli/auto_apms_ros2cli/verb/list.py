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

from auto_apms_behavior_tree_core import get_all_behavior_tree_resources
from auto_apms_ros2cli.verb import VerbExtension


class ListVerb(VerbExtension):
    """List all available behavior resources."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the list verb."""
        pass  # No additional arguments needed for list

    def main(self, *, args):
        """Main function for the list verb."""
        trees = sorted(get_all_behavior_tree_resources(), key=lambda tree: tree.identity.package_name)
        for tree in trees:
            print(str(tree.identity))
        return 0
