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


from collections import defaultdict
from auto_apms_behavior_tree_core.resources import (
    NodeManifestResource,
    get_node_manifest_resource_identities,
)
from ...verb import VerbExtension
from ...api import (
    PrefixFilteredChoicesCompleter,
    NodeManifestFilteredRegistrationNameCompleter,
    add_dynamic_manifest_help_action,
    print_node_details,
)


class ModelVerb(VerbExtension):
    """Inspect behavior tree node models."""

    def add_arguments(self, parser, cli_name):
        add_dynamic_manifest_help_action(parser, "manifest", "node_name")
        manifest_arg = parser.add_argument(
            "manifest",
            type=NodeManifestResource,
            help="Identity string of a node manifest",
        )
        manifest_arg.completer = PrefixFilteredChoicesCompleter(get_node_manifest_resource_identities())
        node_name_arg = parser.add_argument(
            "node_name",
            type=str,
            help="Registration name for one of the nodes from identity",
            nargs="?",
        )
        node_name_arg.completer = NodeManifestFilteredRegistrationNameCompleter("manifest")

    def main(self, *, args):
        # If specific node name is provided, show details for that node only
        if args.node_name:
            if args.node_name in args.manifest.node_model:
                print_node_details(args.manifest, args.node_name)
            else:
                print(f"Node '{args.node_name}' not found in model associated with manifest '{args.manifest.identity}'")
                return 1
        else:
            # Print overview of all nodes in the model and their type
            grouped = defaultdict(list)
            for name, node in args.manifest.node_model.items():
                grouped[node.type].append(name)

            for type_name, node_names in dict(sorted(grouped.items(), key=lambda item: item[0].name)).items():
                print(f"{type_name.name} ({len(node_names)})")
                for node_name in sorted(node_names):
                    # Print each node name with its type
                    print(
                        f"  - {node_name} ({args.manifest.node_manifest.get_node_registration_options(node_name)["class_name"]})"
                    )

        return 0
