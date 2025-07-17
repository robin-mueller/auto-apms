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

from ..verb import VerbExtension
from ..api import _add_behavior_resource_argument_to_parser


class ShowVerb(VerbExtension):
    """Show the content of a behavior resource."""

    def add_arguments(self, parser, cli_name):
        """Add arguments for the show verb."""
        _add_behavior_resource_argument_to_parser(parser)

    def main(self, *, args):
        """Main function for the show verb."""
        print(args.behavior.build_request)
        return 0
