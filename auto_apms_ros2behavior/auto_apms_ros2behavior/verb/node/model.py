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
from auto_apms_behavior_tree_core.tree.node_model import NodePortDirection
from auto_apms_behavior_tree_core.resources import (
    NodeManifestResource,
    get_node_manifest_resource_identities,
)
from ...verb import VerbExtension
from ...api import PrefixFilteredChoicesCompleter, NodeManifestFilteredRegistrationNameCompleter


class ModelVerb(VerbExtension):
    """Inspect behavior tree node models."""

    def add_arguments(self, parser, cli_name):
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
                self._print_node_details(
                    args.manifest.node_model[args.node_name].type.name,
                    args.node_name,
                    args.manifest.node_manifest.get_node_registration_options(args.node_name)["class_name"],
                    args.manifest.node_manifest.get_node_registration_options(args.node_name)["description"],
                    args.manifest.node_model[args.node_name].port_infos,
                )
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

    def _print_node_details(self, node_type, node_name, node_class, desc, ports):
        """Print detailed information for a specific node."""
        print(f"{node_type} {node_name} ({node_class})")
        print(f"Description: {desc if desc.endswith('.') else f"{desc}."}\n")
        if not ports:
            print("No ports defined for this node.")
            return

        # Group ports by direction
        input_ports = [p for p in ports if p.port_direction == NodePortDirection.INPUT]
        output_ports = [p for p in ports if p.port_direction == NodePortDirection.OUTPUT]
        inout_ports = [p for p in ports if p.port_direction == NodePortDirection.INOUT]

        if input_ports:
            print(f"{NodePortDirection.INPUT.name} ({len(input_ports)})")
            for port in input_ports:
                self._print_port_info(port)

        if output_ports:
            print(f"{NodePortDirection.OUTPUT.name} ({len(output_ports)})")
            for port in output_ports:
                self._print_port_info(port)

        if inout_ports:
            print(f"{NodePortDirection.INOUT.name} ({len(inout_ports)})")
            for port in inout_ports:
                self._print_port_info(port)

    def _print_port_info(self, port):
        """Print detailed information for a single port."""
        print(f"  - \033[1m{port.port_name}\033[0m ({port.port_type})")
        if port.port_description:
            print(f"    {port.port_description}")
        if port.port_has_default:
            default_str = port.port_default if port.port_default else "(empty)"
            print(f"    Default: {default_str}")
