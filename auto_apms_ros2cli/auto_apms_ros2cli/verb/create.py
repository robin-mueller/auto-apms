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

"""Create verb for AUTO-APMS behavior tree management."""

from ..verb import VerbExtension


class CreateVerb(VerbExtension):
    """Create a new behavior tree from template."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the create verb."""
        parser.add_argument(
            "name",
            type=str,
            help="Name of the behavior tree to create",
        )
        parser.add_argument(
            "--package",
            type=str,
            help="Target package for the behavior tree (defaults to current package if in one)",
        )
        parser.add_argument(
            "--template",
            type=str,
            choices=["empty", "basic", "navigation"],
            default="empty",
            help="Template to use for the new behavior tree",
        )

    def main(self, *, args):
        """Main function for the create verb."""

        print("Create functionality not yet implemented")
        return 0
