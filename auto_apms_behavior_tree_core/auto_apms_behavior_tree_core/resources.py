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

import ament_index_python
import os
import yaml

from pathlib import Path
from auto_apms_util.resources import *

RESOURCE_TYPE_NAME__BEHAVIOR = "auto_apms_behavior_tree_core__behavior"
RESOURCE_TYPE_NAME__NODE_MANIFEST = "auto_apms_behavior_tree_core__node_manifest"

BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE = "auto_apms_behavior_tree::core::NodeRegistrationInterface"

# Delimiters used in resource identities
RESOURCE_IDENTITY_CATEGORY_SEPARATOR = "/"
RESOURCE_IDENTITY_RESOURCE_SEPARATOR = "::"

# Default behavior categories
DEFAULT_BEHAVIOR_CATEGORY = "default"
DEFAULT_BEHAVIOR_CATEGORY_TREE = "tree"


class ResourceIdentityFormatError(Exception):
    pass


class ResourceError(Exception):
    pass


class NodeManifestError(Exception):
    pass


class NodeManifest:
    """
    Data structure holding information about which behavior tree node plugin to load and how to configure them.

    This is the Python equivalent of auto_apms_behavior_tree::core::NodeManifest.
    A node manifest is a collection of dictionaries mapped by node names.
    """

    def __init__(self, node_dict: dict[str, dict] = None):
        """
        Initialize a node manifest from a dictionary.

        Args:
            node_dict: Dictionary mapping node names to node registration options for manual creation
        """
        self._node_dict: dict[str, dict] = {}
        if node_dict:
            self._node_dict = node_dict.copy()

    @property
    def data(self) -> dict[str, dict]:
        """Get the manifest data as a dictionary of node option dictionaries."""
        return self._node_dict.copy()

    def get_node_names(self) -> list[str]:
        """Get all node names defined in this manifest."""
        return list(self._node_dict.keys())

    def get_node_registration_options(self, node_name: str) -> dict:
        """
        Get registration options for a specific node.

        Args:
            node_name: The name of the node to get options for.

        Returns:
            Dictionary containing node options.

        Raises:
            KeyError: If the node name is not found in the manifest.
        """
        if node_name not in self._node_dict:
            raise KeyError(f"Node '{node_name}' not found in manifest '{self._identity}'")
        return self._node_dict[node_name].copy()

    def has_node(self, node_name: str) -> bool:
        """Check if a node is defined in this manifest."""
        return node_name in self._node_dict

    def add_node(self, node_name: str, options: dict) -> "NodeManifest":
        """
        Add registration options for a behavior tree node to the manifest.

        Args:
            node_name: Name of the behavior tree node.
            options: Dictionary of registration options to be used when loading the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If the registration options are invalid or node_name already exists.
        """
        if not isinstance(options, dict):
            raise NodeManifestError(f"Options for node '{node_name}' must be a dictionary")
        if not options.get("class_name", "").strip():
            raise NodeManifestError(f"Invalid registration options for node '{node_name}': class_name is required")
        if node_name in self._node_dict.keys():
            raise NodeManifestError(f"Node '{node_name}' already exists in manifest")

        self._node_dict[node_name] = options.copy()
        return self

    def remove_node(self, node_name: str) -> "NodeManifest":
        """
        Remove registration options for a behavior tree node.

        Args:
            node_name: Name of the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If node_name doesn't exist.
        """
        if node_name not in self._node_dict:
            raise NodeManifestError(f"Node '{node_name}' not found in manifest")
        del self._node_dict[node_name]
        return self

    def merge(self, other: "NodeManifest", replace: bool = False) -> "NodeManifest":
        """
        Merge another NodeManifest with this one.

        Args:
            other: Other node manifest.
            replace: True for automatically replacing entries with the same key.
                    If False, raises error when other contains existing keys.

        Returns:
            Modified node manifest (self).

        Raises:
            NodeManifestError: If other shares entries and replace is False.
        """
        for node_name, options in other._node_dict.items():
            if node_name in self._node_dict and not replace:
                raise NodeManifestError(f"Node '{node_name}' already exists and replace=False")
            self._node_dict[node_name] = options
        return self

    def size(self) -> int:
        """Get the number of behavior tree nodes this manifest holds registration options for."""
        return len(self._node_dict)

    def empty(self) -> bool:
        """Determine whether any node registration options have been added to the manifest."""
        return len(self._node_dict) == 0

    def to_file(self, file_path: str) -> None:
        """
        Write the node manifest to a YAML file.

        Args:
            file_path: Path to the target file.

        Raises:
            IOError: If file cannot be opened or written.
        """
        try:
            with open(file_path, "w") as f:
                yaml.dump(self._node_dict, f, default_flow_style=False)
        except Exception as e:
            raise IOError(f"Failed to write manifest to file '{file_path}': {e}")

    def dump(self) -> str:
        """
        Dump the node manifest as a YAML string.
        """
        return yaml.safe_dump(self._node_dict)

    @staticmethod
    def from_file(file_path: str) -> "NodeManifest":
        """
        Create a node plugin manifest from a YAML file.

        Args:
            file_path: Path to the manifest file.

        Returns:
            NodeManifest created from the file.

        Raises:
            yaml.error.YAMLError: If the file does not contain a valid YAML mapping.
            OSError: If the file cannot be read.
        """
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
            if not isinstance(data, dict):
                raise yaml.error.YAMLError(f"File '{file_path}' does not contain a valid YAML mapping")

            manifest = NodeManifest()
            for node_name, node_data in data.items():
                if isinstance(node_data, dict):
                    manifest.add_node(node_name, node_data)
                else:
                    raise yaml.error.YAMLError(f"Invalid node data for '{node_name}' in file '{file_path}'")

            return manifest

    @staticmethod
    def from_files(file_paths: list[str]) -> "NodeManifest":
        """
        Create a node plugin manifest from multiple files. They are loaded in the given order.

        Args:
            file_paths: Paths to the manifest files.

        Returns:
            NodeManifest created by merging all files.

        Raises:
            NodeManifestError: If merge fails for any file.
        """
        manifest = NodeManifest()
        for file_path in file_paths:
            try:
                manifest.merge(NodeManifest.from_file(file_path), replace=True)
            except Exception as e:
                raise NodeManifestError(
                    f"Error creating node manifest from multiple files (parsing file {file_path}): {e}"
                ) from e
        return manifest

    @staticmethod
    def from_resource_identity(identity: str) -> "NodeManifest":
        """
        Create a node manifest from an installed resource.

        The resource identity must be specified in the format `<package_name>::<metadata_id>` or simply `<metadata_id>`.

        Args:
            identity: Identity of the node manifest resource.

        Returns:
            Node manifest created from the corresponding resource.

        Raises:
            ResourceIdentityFormatError: If identity has wrong format.
            ResourceError: If resource cannot be determined using identity.
        """
        tokens = identity.split(RESOURCE_IDENTITY_RESOURCE_SEPARATOR)
        package_name = ""
        file_stem = ""

        if len(tokens) == 1:
            file_stem = tokens[0]
        elif len(tokens) == 2:
            package_name = tokens[0]
            file_stem = tokens[1]
        else:
            raise ResourceIdentityFormatError(
                f"Node manifest resource identity string '{identity}' has wrong format. "
                f"Must be '<package_name>{RESOURCE_IDENTITY_RESOURCE_SEPARATOR}<metadata_id>'."
            )

        search_packages: set[str] = set()
        if package_name:
            search_packages.add(package_name)
        else:
            search_packages = set(ament_index_python.get_resources(RESOURCE_TYPE_NAME__NODE_MANIFEST))

        matching_file_paths: list[str] = []
        for package in search_packages:
            content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__NODE_MANIFEST, package)
            lines = content.splitlines()
            for line in lines:
                parts = line.split("|")
                if len(parts) != 2:
                    raise ResourceError(
                        f"Invalid node manifest resource file (Package: '{package}'). Invalid line: {line}."
                    )
                if parts[0] == file_stem:
                    matching_file_paths.append(os.path.join(base_path, parts[1]))

        if not matching_file_paths:
            raise ResourceError(f"No node manifest resource was found using identity '{identity}'.")
        if len(matching_file_paths) > 1:
            raise ResourceError(f"There are multiple node manifest resources with metadata ID '{file_stem}'.")

        return NodeManifest.from_files([matching_file_paths[0]])


class BehaviorResourceIdentity:
    """
    Base class that encapsulates the identity string for a registered behavior.

    This is the Python equivalent of auto_apms_behavior_tree::core::BehaviorResourceIdentity.
    """

    def __init__(self, identity: str = None):
        """
        Initialize a behavior resource identity from an identity string.

        Identity must be formatted like `<category_name>/<package_name>::<resource_name>`.
        Both category_name and package_name are optional.

        Args:
            identity: Identity string for a specific behavior resource, or None for manual creation.
        """
        self.category_name = ""
        self.package_name = ""
        self.behavior_alias = ""

        if identity is None:
            return

        # Parse category and resource parts
        behavior_alias_part = identity
        if RESOURCE_IDENTITY_CATEGORY_SEPARATOR in identity:
            category_pos = identity.find(RESOURCE_IDENTITY_CATEGORY_SEPARATOR)
            self.category_name = identity[:category_pos]
            behavior_alias_part = identity[category_pos + len(RESOURCE_IDENTITY_CATEGORY_SEPARATOR) :]

        # Parse package and resource name
        if RESOURCE_IDENTITY_RESOURCE_SEPARATOR in behavior_alias_part:
            separator_pos = behavior_alias_part.find(RESOURCE_IDENTITY_RESOURCE_SEPARATOR)
            self.package_name = behavior_alias_part[:separator_pos]
            self.behavior_alias = behavior_alias_part[separator_pos + len(RESOURCE_IDENTITY_RESOURCE_SEPARATOR) :]
        else:
            # If only a single token is given, assume it's behavior_alias
            self.package_name = ""
            self.behavior_alias = behavior_alias_part

        if not self.package_name and not self.behavior_alias:
            raise ResourceIdentityFormatError(
                f"Behavior resource identity string '{identity}' is invalid. Package and resource name must not be empty."
            )

    def __str__(self) -> str:
        """Create the corresponding identity string."""
        result = ""
        if self.category_name:
            result += self.category_name + RESOURCE_IDENTITY_CATEGORY_SEPARATOR
        result += self.package_name + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + self.behavior_alias
        return result

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other: "BehaviorResourceIdentity") -> bool:
        return str(self) == str(other)

    def __lt__(self, other: "BehaviorResourceIdentity") -> bool:
        return str(self) < str(other)

    def empty(self) -> bool:
        """Determine whether this behavior resource identity object is considered empty."""
        return not self.package_name and not self.behavior_alias


class BehaviorResource:
    """
    Class containing behavior resource data.

    This is the Python equivalent of auto_apms_behavior_tree::core::BehaviorResource.
    """

    def __init__(self, identity: BehaviorResourceIdentity | str):
        """
        Initialize a behavior resource using an identity.

        Args:
            identity: BehaviorResourceIdentity object or identity string.
        """
        if isinstance(identity, str):
            self._identity = BehaviorResourceIdentity(identity)
        elif isinstance(identity, BehaviorResourceIdentity):
            self._identity = identity
        else:
            raise TypeError("Identity must be a BehaviorResourceIdentity object or a string.")

        # Load resource data from ament index
        self._package = ""
        self._category = ""
        self._build_request = ""
        self._build_request_file_path = ""
        self._default_build_handler = ""
        self._entrypoint = ""
        self._node_manifest = NodeManifest()

        # Find the resource in the ament index - search across all packages if needed
        search_packages: set[str] = set()
        if self._identity.package_name:
            search_packages.add(self._identity.package_name)
        else:
            search_packages = get_packages_with_resource_type(RESOURCE_TYPE_NAME__BEHAVIOR)

        matching_count = 0
        for package in search_packages:
            content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__BEHAVIOR, package)

            # Parse content to find the specific resource
            for line in content.splitlines():
                parts = line.split("|")
                if len(parts) != 6:
                    raise ResourceError(f"Invalid behavior resource file (Package: '{package}'). Invalid line: {line}.")

                # Store behavior category
                self._category = parts[0]

                # Determine if resource is matching
                alias = parts[1]
                if not self._identity.category_name:
                    # Disregard the category if not provided with the identity
                    if alias != self._identity.behavior_alias:
                        continue
                elif self._identity.category_name != self._category or alias != self._identity.behavior_alias:
                    continue

                # Found matching resource - Increase counter
                matching_count += 1

                # Store package name
                self._package = package

                # Store default build handler
                self._default_build_handler = parts[2]

                # Store build request
                self._build_request_file_path = os.path.join(base_path, parts[3])
                if os.path.isfile(self._build_request_file_path):
                    with open(self._build_request_file_path, "r") as f:
                        self._build_request = f.read()
                else:
                    self._build_request_file_path = ""
                    self._build_request = parts[3]

                # Store entrypoint
                self._entrypoint = parts[4]

                # Store node manifest
                node_manifest_paths = []
                for path in parts[5].split(";"):
                    if os.path.isabs(path):
                        node_manifest_paths.append(path)
                    else:
                        node_manifest_paths.append(os.path.join(base_path, path))
                self._node_manifest = NodeManifest.from_files(node_manifest_paths)

        if matching_count == 0:
            raise ResourceError(f"No behavior resource with identity '{self._identity}' was registered.")
        if matching_count > 1:
            raise ResourceError(
                f"Behavior resource identity '{self._identity}' is ambiguous. You must be more precise."
            )

    @staticmethod
    def find(behavior_alias: str, package_name: str = "", category_name: str = "") -> "BehaviorResource":
        """
        Find a behavior resource by name.

        Args:
            behavior_alias: Name of the resource to find.
            package_name: Optional package name to narrow search.
            category_name: Optional category name to narrow search.

        Returns:
            BehaviorResource instance for the found resource.

        Raises:
            ResourceError: If resource cannot be found.
        """
        return BehaviorResource(
            category_name
            + RESOURCE_IDENTITY_CATEGORY_SEPARATOR
            + package_name
            + RESOURCE_IDENTITY_RESOURCE_SEPARATOR
            + behavior_alias
        )

    @property
    def category_name(self) -> str:
        return self._category

    @property
    def package_name(self) -> str:
        return self._package

    @property
    def build_request(self) -> str:
        return self._build_request

    @property
    def default_build_handler(self) -> str:
        return self._default_build_handler

    @property
    def entrypoint(self) -> str:
        return self._entrypoint

    @property
    def node_manifest(self) -> NodeManifest:
        return self._node_manifest


class TreeResourceIdentity(BehaviorResourceIdentity):
    """
    Struct that encapsulates the identity string for a declared behavior tree.

    This is the Python equivalent of auto_apms_behavior_tree::core::TreeResourceIdentity.
    """

    def __init__(self, identity: str = None):
        """
        Initialize a tree resource identity from an identity string.

        Identity must be formatted like `<package_name>::<tree_file_stem>::<tree_name>`.

        Args:
            identity: Identity string for a specific behavior tree resource, or None for manual creation.
        """
        self.file_stem = ""
        self.tree_name = ""

        if identity is None:
            super().__init__()
            return

        # First, let the parent class parse the basic structure
        super().__init__(identity)

        # If no category is explicitly specified, use a special default for behavior trees
        if not self.category_name or self.category_name == DEFAULT_BEHAVIOR_CATEGORY:
            self.category_name = DEFAULT_BEHAVIOR_CATEGORY_TREE

        # Parse the resource_name part to extract file_stem and tree_name
        tokens = self.behavior_alias.split(RESOURCE_IDENTITY_RESOURCE_SEPARATOR)

        # If only a single token is given, assume it's file_stem
        if len(tokens) == 1:
            tokens.append("")

        if len(tokens) != 2:
            raise ResourceIdentityFormatError(
                f"Tree resource identity string '{identity}' is invalid. "
                f"Resource name must contain 2 tokens (separated by {RESOURCE_IDENTITY_RESOURCE_SEPARATOR})."
            )

        self.file_stem = tokens[0]
        self.tree_name = tokens[1]

        if not self.file_stem and not self.tree_name:
            raise ResourceIdentityFormatError(
                f"Behavior tree resource identity string '{identity}' is invalid. "
                "It's not allowed to omit both <tree_file_stem> and <tree_name>."
            )


class TreeResource(BehaviorResource):
    """
    Class containing behavior tree resource data.

    This is the Python equivalent of auto_apms_behavior_tree::core::TreeResource.
    """

    def __init__(self, identity: TreeResourceIdentity | str):
        """Initialize a tree resource from an identity object or string."""
        if isinstance(identity, str):
            self._identity = TreeResourceIdentity(identity)
        else:
            self._identity = identity

        # Initialize the base class
        super().__init__(self._identity)

    @staticmethod
    def find_by_tree_name(tree_name: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using a specific behavior tree name.

        Args:
            tree_name (str): The name of the behavior tree.
            package_name (str, optional): The name of the package to search in. Default: Search in all installed packages.

        Returns:
            TreeResource: The matching behavior tree resource.
        """
        return TreeResource(
            package_name + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + tree_name
        )

    @staticmethod
    def find_by_file_stem(file_stem: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using a specific behavior tree XML file stem.

        Args:
            file_stem (str): The file stem of the behavior tree XML file.
            package_name (str, optional): The name of the package to search in. Default: Search in all installed packages.

        Returns:
            TreeResource: The matching behavior tree resource.
        """
        return TreeResource(
            package_name + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + file_stem + RESOURCE_IDENTITY_RESOURCE_SEPARATOR
        )

    @property
    def file_stem(self) -> str:
        """
        Get the file stem of the behavior tree resource.
        """
        return Path(self._build_request_file_path).stem


# TODO: Refactor so the resource parsing is only done in one function and not also in the resource constructor
def get_all_behavior_resources(
    categories: list[str] = None, exclude_packages: set[str] = None
) -> dict[BehaviorResourceIdentity, BehaviorResource]:
    """
    Retrieves all available/installed behavior resources.

    This function scans all packages that have registered behavior resources,
    parses their resource files, and constructs a list of `BehaviorResource` objects
    representing each available behavior.

    Args:
        categories: Optional categories to filter behavior resources by. If omitted, all categories are included.
        exclude_packages: Packages to exclude when searching for resources.

    Returns:
        A dictionary of behavior resources mapped with their identities.

    Raises:
        ValueError: If a resource file contains an invalid line.
    """
    behaviors: dict[BehaviorResourceIdentity, BehaviorResource] = {}
    for package in get_packages_with_resource_type(RESOURCE_TYPE_NAME__BEHAVIOR, exclude_packages):
        content, _ = ament_index_python.get_resource(RESOURCE_TYPE_NAME__BEHAVIOR, package)
        for line in content.splitlines():
            parts = line.split("|")
            if len(parts) != 5:
                raise ValueError(f"Invalid behavior resource file (Package: '{package}'). Invalid line: {line}.")

            found_category = parts[0]
            if categories and found_category not in categories:
                continue
            behavior_alias = parts[1]
            identity = BehaviorResourceIdentity(
                found_category
                + RESOURCE_IDENTITY_CATEGORY_SEPARATOR
                + package
                + RESOURCE_IDENTITY_RESOURCE_SEPARATOR
                + behavior_alias
            )
            behaviors[identity] = BehaviorResource(identity)

    return dict(sorted(behaviors.items()))


def get_all_tree_node_plugins(exclude_packages: set[str] = None) -> list[str]:
    """
    Get all behavior tree node plugin names.

    This is a convenience function that finds all plugins with the behavior tree node
    base class type.

    Args:
        exclude_packages: Packages to exclude when searching for plugins.

    Returns:
        list[str]: List of all behavior tree node plugin names.

    Raises:
        ResourceError: If failed to find or parse plugin manifest files.
    """
    return get_plugin_names_with_base_type(BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE, exclude_packages)


def get_all_tree_node_manifest_resources(exclude_packages: set[str] = None) -> dict[str, NodeManifest]:
    """
    Get all available node manifest resources.

    Args:
        exclude_packages: Packages to exclude when searching for manifests.

    Returns:
        Dictionary of all node manifest resources with their identities as keys.

    Raises:
        ResourceError: If failed to find or parse manifest files.
    """
    manifests: dict[str, NodeManifest] = {}
    for package in get_packages_with_resource_type(RESOURCE_TYPE_NAME__NODE_MANIFEST, exclude_packages):
        content, _ = ament_index_python.get_resource(RESOURCE_TYPE_NAME__NODE_MANIFEST, package)
        for line in content.splitlines():
            # Parse resource line format: metadata_id|manifest_path
            parts = line.split("|")
            if len(parts) != 2:
                raise ResourceError(
                    f"Invalid node manifest resource file (Package: '{package}'). Invalid line: {line}."
                )
            metadata_id = parts[0]
            identity = package + RESOURCE_IDENTITY_RESOURCE_SEPARATOR + metadata_id
            manifests[identity] = NodeManifest.from_resource_identity(identity)

    return manifests
