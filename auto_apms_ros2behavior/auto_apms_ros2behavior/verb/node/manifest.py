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

import yaml

from auto_apms_behavior_tree_core.resources import (
    NodeManifest,
    NodeManifestResourceIdentity,
    get_node_manifest_resource_identities,
)
from ...verb import VerbExtension
from ...api import PrefixFilteredChoicesCompleter, NodeManifestFilteredRegistrationNameCompleter


class ManifestVerb(VerbExtension):
    """Inspect registered behavior tree node manifests."""

    def add_arguments(self, parser, cli_name):
        self.identities = get_node_manifest_resource_identities()
        identity_arg = parser.add_argument(
            "identity",
            type=NodeManifestResourceIdentity,
            help="Identity string of a node manifest to inspect",
            nargs="?",
        )
        identity_arg.completer = PrefixFilteredChoicesCompleter(self.identities)
        node_name_arg = parser.add_argument(
            "node_name",
            type=str,
            help="Registration name for one of the nodes from identity",
            nargs="?",
        )
        node_name_arg.completer = NodeManifestFilteredRegistrationNameCompleter("identity")

    def main(self, *, args):
        if args.identity:
            self.identities = [args.identity]
            if args.node_name:
                # If a specific node name is given, print its details
                manifest = NodeManifest.from_resource(args.identity)
                print(
                    yaml.dump(
                        {args.node_name: manifest.get_node_registration_options(args.node_name)},
                        indent=2,
                        sort_keys=False,
                    ),
                    end="",
                )
                return 0

        # If no arguments specified, list manifests
        for i in self.identities:
            print(str(i))
            manifest = NodeManifest.from_resource(i)
            for name in manifest.get_node_names():
                print(f"  - {name} ({manifest.get_node_registration_options(name)["class_name"]})")
        return 0
