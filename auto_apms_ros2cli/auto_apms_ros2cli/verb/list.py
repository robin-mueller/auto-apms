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
    get_behavior_resource_identities,
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP,
)
from ..verb import VerbExtension
from ..api import PrefixFilteredChoicesCompleter


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
        categories_arg.completer = PrefixFilteredChoicesCompleter(
            {i.category_name for i in get_behavior_resource_identities()}
        )
        parser.add_argument(
            "--include-internal",
            action="store_true",
            help="Flag whether to include behaviors marked as internal.",
        )

    def main(self, *, args):
        """Main function for the list verb."""
        identities = get_behavior_resource_identities(args.categories, args.include_internal)
        print("Total:", len(identities))

        # Group behaviors by category
        categorized_behaviors = defaultdict(list)
        for i in identities:
            categorized_behaviors[i.category_name].append(
                str(i).split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP, maxsplit=1)[-1]
            )

        # Print grouped behaviors
        for category, items in categorized_behaviors.items():
            print(f"{category}{_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP}")
            for cat_i in items:
                print(f"    {cat_i}")

        return 0
