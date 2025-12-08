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

from auto_apms_behavior_tree_core.resources import (
    get_behavior_resource_identities,
    get_behavior_categories,
)
from ..verb import VerbExtension
from ..api import PrefixFilteredChoicesCompleter, print_grouped_behavior_identities


class ListVerb(VerbExtension):
    """List all available behavior resources."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the list verb."""
        categories_arg = parser.add_argument(
            "-c",
            "--categories",
            nargs="*",
            help="List all behavior resources in the specified categories. If no category is given, all resources are listed.",
        )
        categories_arg.completer = PrefixFilteredChoicesCompleter(get_behavior_categories())
        parser.add_argument(
            "--include-internal",
            action="store_true",
            help="Flag whether to include behaviors marked as internal.",
        )
        parser.add_argument(
            "--group-by",
            choices=["category", "package"],
            default="category",
            help="How to group the behavior resources. Default is 'category'.",
        )

    def main(self, *, args):
        """Main function for the list verb."""
        identities = get_behavior_resource_identities(args.categories, args.include_internal)
        print_grouped_behavior_identities(identities, args.group_by)
        return 0
