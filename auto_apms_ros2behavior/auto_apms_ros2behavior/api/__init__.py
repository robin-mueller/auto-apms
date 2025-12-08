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

from typing import Literal
from collections import defaultdict
from argparse import ArgumentParser, Action
from argcomplete.completers import BaseCompleter, ChoicesCompleter
from auto_apms_behavior_tree_core.resources import (
    BehaviorResource,
    BehaviorResourceIdentity,
    NodeManifest,
    NodeManifestResource,
    NodeManifestResourceIdentity,
    get_behavior_resource_identities,
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP,
    _AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_ALIAS_SEP,
)
from auto_apms_behavior_tree_core.tree.node_model import NodePortDirection, NodePortInfo


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


def print_port_info(port: NodePortInfo):
    """Print detailed information for a single port."""
    print(f"  - \033[1m{port.port_name}\033[0m ({port.port_type})")
    if port.port_description:
        print(f"    {port.port_description}")
    if port.port_has_default:
        default_str = port.port_default if port.port_default else "(empty)"
        print(f"    Default: {default_str}")


def print_node_port_infos(port_infos: list[NodePortInfo]):
    """
    Print a list of NodePortInfo objects grouped by direction.

    Args:
        port_infos: List of NodePortInfo objects.
    """
    if not port_infos:
        print("No ports defined for this node.")
        return

    # Group ports by direction
    input_ports = [p for p in port_infos if p.port_direction == NodePortDirection.INPUT]
    output_ports = [p for p in port_infos if p.port_direction == NodePortDirection.OUTPUT]
    inout_ports = [p for p in port_infos if p.port_direction == NodePortDirection.INOUT]

    if input_ports:
        print(f"{NodePortDirection.INPUT.name} ({len(input_ports)})")
        for port in input_ports:
            print_port_info(port)

    if output_ports:
        print(f"{NodePortDirection.OUTPUT.name} ({len(output_ports)})")
        for port in output_ports:
            print_port_info(port)

    if inout_ports:
        print(f"{NodePortDirection.INOUT.name} ({len(inout_ports)})")
        for port in inout_ports:
            print_port_info(port)


def print_node_details(manifest: NodeManifestResource, node_name: str):
    """Print detailed information for a specific node."""
    desc = manifest.node_manifest.get_node_registration_options(node_name)["description"]
    print(
        f"{manifest.node_model[node_name].type.name} {node_name} ({manifest.node_manifest.get_node_registration_options(node_name)["class_name"]})"
    )
    print(f"Description: {desc if desc.endswith('.') else f"{desc}."}\n")
    print_node_port_infos(manifest.node_model[node_name].port_infos)


def print_manifest_node_names(node_manifest: NodeManifest):
    """
    Print all node names from a manifest with their class names.

    Args:
        node_manifest: The NodeManifest to print nodes from.
    """
    for name in node_manifest.get_node_names():
        print(f"  - {name} ({node_manifest.get_node_registration_options(name)['class_name']})")


def remove_default_help_action(parser):
    """Remove the default help action from a parser to allow replacing it."""
    for action in list(parser._actions):
        if action.option_strings == ["-h", "--help"]:
            parser._actions.remove(action)
            # Also remove from option_string_actions dict
            for opt_str in action.option_strings:
                if opt_str in parser._option_string_actions:
                    del parser._option_string_actions[opt_str]
            # Also remove from the action group
            for group in parser._action_groups:
                if action in group._group_actions:
                    group._group_actions.remove(action)
            break


class DynamicPortHelpAction(Action):
    """
    Custom argparse action that displays dynamic help including available ports
    based on previously parsed manifest and node_name arguments.
    """

    def __init__(self, option_strings, dest, manifest_arg_name, node_name_arg_name, **kwargs):
        self._manifest_arg_name = manifest_arg_name
        self._node_name_arg_name = node_name_arg_name
        super().__init__(option_strings, dest, nargs=0, default=None, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        # Print the standard help first
        parser.print_help()

        # Try to get manifest and node_name from namespace (parsed so far)
        manifest = getattr(namespace, self._manifest_arg_name, None)
        node_name = getattr(namespace, self._node_name_arg_name, None)

        if manifest:
            try:
                if isinstance(manifest, (str, NodeManifestResourceIdentity)):
                    manifest = NodeManifestResource(manifest)
                if isinstance(manifest, NodeManifestResource):
                    if node_name:
                        # Show node details
                        print(f"\nAvailable ports for ", end="")
                        print_node_details(manifest, node_name)
                    else:
                        # Show available nodes in the manifest
                        print(f"\nAvailable nodes in '{manifest.identity}':")
                        print_manifest_node_names(manifest.node_manifest)
            except Exception:
                pass  # Silently ignore errors in dynamic help generation

        parser.exit()


class DynamicManifestHelpAction(Action):
    """
    Custom argparse action that displays dynamic help including available nodes
    based on previously parsed manifest argument.
    """

    def __init__(self, option_strings, dest, manifest_arg_name, node_name_arg_name=None, **kwargs):
        self._manifest_arg_name = manifest_arg_name
        self._node_name_arg_name = node_name_arg_name
        super().__init__(option_strings, dest, nargs=0, default=None, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        # Print the standard help first
        parser.print_help()

        # Try to get manifest from namespace (parsed so far)
        manifest = getattr(namespace, self._manifest_arg_name, None)
        node_name = getattr(namespace, self._node_name_arg_name, None) if self._node_name_arg_name else None

        if manifest and not node_name:
            try:
                if isinstance(manifest, (str, NodeManifestResourceIdentity)):
                    node_manifest = NodeManifest.from_resource(manifest)
                elif isinstance(manifest, NodeManifestResource):
                    node_manifest = manifest.node_manifest
                else:
                    node_manifest = None

                if node_manifest:
                    print(f"\nAvailable nodes in '{manifest}':")
                    print_manifest_node_names(node_manifest)
            except Exception:
                pass  # Silently ignore errors in dynamic help generation

        parser.exit()


def add_dynamic_node_help_action(parser, manifest_arg_name, node_name_arg_name):
    """
    Replace the default help action with a dynamic help action that shows
    available nodes or ports based on parsed arguments.

    Args:
        parser: The argument parser to modify.
        manifest_arg_name: Name of the manifest argument.
        node_name_arg_name: Name of the node_name argument.
    """
    remove_default_help_action(parser)
    parser.add_argument(
        "-h",
        "--help",
        action=DynamicPortHelpAction,
        manifest_arg_name=manifest_arg_name,
        node_name_arg_name=node_name_arg_name,
        help="Show this help message (with node/port details if arguments are provided)",
    )


def add_dynamic_manifest_help_action(parser, manifest_arg_name, node_name_arg_name=None):
    """
    Replace the default help action with a dynamic help action that shows
    available nodes based on the parsed manifest argument.

    Args:
        parser: The argument parser to modify.
        manifest_arg_name: Name of the manifest argument.
        node_name_arg_name: Optional name of the node_name argument.
    """
    remove_default_help_action(parser)
    parser.add_argument(
        "-h",
        "--help",
        action=DynamicManifestHelpAction,
        manifest_arg_name=manifest_arg_name,
        node_name_arg_name=node_name_arg_name,
        help="Show this help message (with node list if manifest is provided)",
    )


def print_grouped_behavior_identities(
    identities: list[BehaviorResourceIdentity], group_by: Literal["category", "package"]
):
    """
    Print behavior resource identities.

    Args:
        identities: List of BehaviorResourceIdentity objects.
        group_by: How to group the identities.
    """
    if group_by == "category":
        categorized_behaviors = defaultdict(list)
        for i in identities:
            categorized_behaviors[i.category_name].append(
                str(i).split(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP, maxsplit=1)[-1]
            )
        for category, items in categorized_behaviors.items():
            print(f"{category}{_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_IDENTITY_CATEGORY_SEP}")
            for cat_i in items:
                print(f"  - {cat_i}")
        return
    if group_by == "package":
        categorized_behaviors = defaultdict(list)
        for i in identities:
            categorized_behaviors[i.package_name].append(i)
        for package, items in categorized_behaviors.items():
            print(f"Package: {package}")
            for pkg_i in items:
                print(f"  - {pkg_i}")
        return


def add_behavior_resource_argument_to_parser(parser: ArgumentParser):
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
