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
from auto_apms_behavior_tree_core import get_all_behavior_tree_resources
from auto_apms_ros2cli.verb import VerbExtension
from ..api import sync_run_behavior_locally, parse_key_value_args, PrefixFilteredChoicesCompleter


class RunVerb(VerbExtension):
    """Execute a behavior locally."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the run verb."""
        tree_id_arg = parser.add_argument(
            "tree_id",
            type=str,
            help="Tree identifier in format <package>::<file_stem>::<tree_name>",
        )
        tree_resources = get_all_behavior_tree_resources()
        trees = [str(tree.identity) for tree in tree_resources]
        tree_id_arg.completer = PrefixFilteredChoicesCompleter(trees)

        parser.add_argument(
            "--blackboard",
            nargs="*",
            metavar="key:=value",
            help="Blackboard variables to pass to the behavior tree",
            default=[],
        )
        parser.add_argument(
            "--tick-rate",
            type=float,
            help="Tick rate for the behavior tree in seconds",
        )
        parser.add_argument(
            "--groot2-port",
            type=int,
            help="Port for Groot2 (disabled by default)",
        )
        parser.add_argument(
            "--state-change-logger",
            action="store_true",
            help="Enable the state change logger",
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
        """Main function for the run verb."""
        # Collect static parameters if specified
        static_params = {}
        if args.tick_rate is not None:
            static_params["tick_rate"] = args.tick_rate
        if args.groot2_port is not None:
            static_params["groot2_port"] = args.groot2_port
        if args.state_change_logger is not None:
            static_params["state_change_logger"] = args.state_change_logger

        # Parse blackboard key-value pairs from args.blackboard
        blackboard_params = parse_key_value_args(args.blackboard)

        return sync_run_behavior_locally(
            tree_id=args.tree_id,
            static_params=static_params,
            blackboard_params=blackboard_params,
            logging_level=args.logging,
        )
