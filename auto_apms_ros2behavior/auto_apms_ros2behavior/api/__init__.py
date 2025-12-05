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

from collections import defaultdict
from argparse import ArgumentParser
from argcomplete.completers import BaseCompleter, ChoicesCompleter
from auto_apms_behavior_tree_core.resources import (
    BehaviorResource,
    NodeManifest,
    NodeManifestResource,
    NodeManifestResourceIdentity,
    get_behavior_resource_identities,
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP,
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP,
)
from auto_apms_behavior_tree_core.tree.node_model import NodePortInfo


class PrefixFilteredChoicesCompleter(ChoicesCompleter):
    """A completer that filters choices based on prefix matching."""

    def __init__(self, choices, *args, **kwargs):
        super().__init__((self._convert(c) for c in choices), *args, **kwargs)

    def __call__(self, prefix, **kwargs):
        return (c for c in self.choices if c.startswith(prefix))


class BehaviorResourceChoicesCompleter(BaseCompleter):
    """
    A completer that filters AutoAPMS behavior resources.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.identity_strings_fully_qualified = [str(i) for i in get_behavior_resource_identities()]
        self.identity_strings_without_category = [
            i.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP, maxsplit=1)[-1]
            for i in self.identity_strings_fully_qualified
        ]
        self.identity_strings_only_behavior_alias = [
            i.split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP, maxsplit=1)[-1]
            for i in self.identity_strings_fully_qualified
        ]

        # Filter ambiguous values (values that would refer to the same identity with the version without category)
        value_counts = defaultdict(int)
        for v in self.identity_strings_without_category:
            value_counts[v] += 1
        self.identity_strings_without_category = [
            i for i in self.identity_strings_without_category if value_counts[v] == 1
        ]

    def __call__(self, prefix, **kwargs):
        choices = self.identity_strings_fully_qualified
        if not prefix:
            return choices

        # Allow users to specify an identity without the category part
        choices_without_category = [c for c in self.identity_strings_without_category if c.startswith(prefix)]
        if any(choices_without_category):
            return choices_without_category
        return choices


class NodeManifestFilteredRegistrationNameCompleter(BaseCompleter):
    def __init__(self, manifest_arg_name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._manifest_arg_name = manifest_arg_name

    def __call__(self, prefix, parsed_args, **kwargs):
        node_manifest = getattr(parsed_args, self._manifest_arg_name, None)
        if not node_manifest:
            return ()
        if isinstance(node_manifest, (str, NodeManifestResourceIdentity)):
            node_manifest = NodeManifest.from_resource(node_manifest)
        if isinstance(node_manifest, NodeManifestResource):
            node_manifest = node_manifest.node_manifest
        if not isinstance(node_manifest, NodeManifest):
            return ()
        return (name for name in node_manifest.get_node_names() if name.startswith(prefix))


class NodePortValuesCompleter(BaseCompleter):
    def __init__(self, manifest_arg_name, node_name_arg_name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._manifest_arg_name = manifest_arg_name
        self._node_name_arg_name = node_name_arg_name

    def __call__(self, prefix, parsed_args, **kwargs):
        node_manifest_resource = getattr(parsed_args, self._manifest_arg_name, None)
        node_name = getattr(parsed_args, self._node_name_arg_name, None)
        if not node_manifest_resource:
            return ()
        if not node_name:
            return ()
        if isinstance(node_manifest_resource, (str, NodeManifestResourceIdentity)):
            node_manifest_resource = NodeManifestResource(node_manifest_resource)
        if not isinstance(node_manifest_resource, NodeManifestResource):
            return ()
        model = node_manifest_resource.node_model[node_name]
        return (self._get_suggestion(info) for info in model.port_infos if info.port_name.startswith(prefix))

    @staticmethod
    def _get_suggestion(port_info: NodePortInfo):
        return (
            f"{port_info.port_name}:={port_info.port_default}"
            if port_info.port_has_default
            else f"{port_info.port_name}:="
        )


def _add_behavior_resource_argument_to_parser(parser: ArgumentParser):
    """
    Add a behavior resource argument to the parser.

    Args:
        parser: The argument parser to add the argument to.
    """
    behavior_arg = parser.add_argument(
        "behavior",
        type=BehaviorResource,
        help="Behavior resource identity string",
        metavar=f"<category_name>{_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP}"
        f"<package_name>{_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP}"
        f"<behavior_alias>",
    )
    behavior_arg.completer = BehaviorResourceChoicesCompleter()
    return behavior_arg


def parse_key_value_args(argv):
    """
    Parse arbitrary key:=value arguments from argv.

    Returns:
        dict: dict of key-value pairs with values converted to builtin Python types.
    """
    parsed_args = {}
    for argument in argv:
        count = argument.count(":=")
        if count == 0 or argument.startswith(":=") or (count == 1 and argument.endswith(":=")):
            raise RuntimeError(f"Malformed key-value argument '{argument}', expected format '<name>:=<value>'")
        key, value = argument.split(":=", maxsplit=1)
        # last one wins is intentional
        try:
            parsed_args[key] = yaml.safe_load(value)
        except Exception:
            parsed_args[key] = value
    return parsed_args
