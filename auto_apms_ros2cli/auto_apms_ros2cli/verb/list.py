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
from auto_apms_behavior_tree_core.resources import get_all_behavior_resources, RESOURCE_IDENTITY_CATEGORY_SEPARATOR
from ..verb import VerbExtension


class ListVerb(VerbExtension):
    """List all available behavior resources."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the list verb."""
        pass  # No additional arguments needed for list

    def main(self, *, args):
        """Main function for the list verb."""
        behaviors = get_all_behavior_resources()
        print("Total:", len(behaviors))

        # Group behaviors by category
        categorized_resources = defaultdict(list)
        for identity, resource in behaviors.items():
            categorized_resources[resource.category_name].append(
                str(identity).split(RESOURCE_IDENTITY_CATEGORY_SEPARATOR, 2)[-1]
            )

        # Print grouped behaviors
        for category, items in categorized_resources.items():
            print(f"{category}{RESOURCE_IDENTITY_CATEGORY_SEPARATOR}")
            for identity in items:
                print(f"    {identity}")

        return 0
