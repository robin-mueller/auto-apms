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


from rclpy.logging import LoggingSeverity, get_logging_severity_from_string
from auto_apms_behavior_tree_core.resources import NodeManifest, get_node_manifest_resource_identities
from ...verb import VerbExtension
from ...api import PrefixFilteredChoicesCompleter, NodeManifestFilteredRegistrationNameCompleter, sync_run_tree_node_locally, parse_key_value_args


class CallVerb(VerbExtension):
    """List all available behavior resources."""

    def add_arguments(self, parser, cli_name):
        manifest_arg_name = "manifest"
        manifest_arg = parser.add_argument(
            manifest_arg_name,
            type=NodeManifest.from_resource,
            help="The identity of the node manifest to use for registering node_name.",
        )
        manifest_arg.completer = PrefixFilteredChoicesCompleter(get_node_manifest_resource_identities())
        node_name_arg_name = "node_name"
        node_name_arg = parser.add_argument(
            node_name_arg_name,
            type=str,
            help="The registration name of the node to call.",
        )
        node_name_arg.completer = NodeManifestFilteredRegistrationNameCompleter(manifest_arg_name)
        parser.add_argument(
            "port_values",
            nargs="*",
            metavar="port_name:=value",
            help="Port values to pass to the node",
            default=[],
        )
        logging_level_names = [enum.name.lower() for enum in LoggingSeverity]
        logging_arg = parser.add_argument(
            "--logging",
            type=get_logging_severity_from_string,
            choices=LoggingSeverity,
            help="Set the logger level for the executor node before starting the tree",
            metavar=logging_level_names,
        )
        logging_arg.completer = PrefixFilteredChoicesCompleter(logging_level_names)

    def main(self, *, args):
        return sync_run_tree_node_locally(
            node_name=args.node_name,
            registration_options=args.manifest.get_node_registration_options(args.node_name),
            port_values=parse_key_value_args(args.port_values),
            logging_level=args.logging,
        )
