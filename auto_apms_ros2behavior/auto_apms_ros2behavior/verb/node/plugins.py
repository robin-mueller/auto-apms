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


from auto_apms_behavior_tree_core.resources import get_behavior_tree_node_plugins
from ...verb import VerbExtension


class PluginsVerb(VerbExtension):
    """List all available behavior tree node plugins."""

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        names_per_package = get_behavior_tree_node_plugins(per_package=True)
        for package, plugins in names_per_package.items():
            print(f"Package: {package}")
            for plugin in plugins:
                print(f"  - {plugin}")
        return 0
