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
from dataclasses import dataclass
from pathlib import Path

from auto_apms_util.resources import *

RESOURCE_TYPE_NAME__TREE = "auto_apms_behavior_tree_core__tree"
RESOURCE_TYPE_NAME__NODE_MANIFEST = "auto_apms_behavior_tree_core__node_manifest"

BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE = "auto_apms_behavior_tree::core::NodeRegistrationInterface"


class ResourceIdentityFormatError(Exception):
    pass


class ResourceError(Exception):
    pass


class NodeManifest:
    """
    Data structure for information about which behavior tree node plugin to load and how to configure them.

    This is the Python equivalent of auto_apms_behavior_tree::core::NodeManifest.
    A node manifest is a collection of dictionaries mapped by node names.
    """

    def __init__(self, identity: str = None, node_map: dict[str, dict] = None):
        """
        Initialize a node manifest from an identity string or from a node map.

        Args:
            identity: Identity to load from resources (format: "package::metadata_id" or "metadata_id"), or None for manual creation
            node_map: Dictionary mapping node names to node option dictionaries for manual creation
        """
        self._node_map: dict[str, dict] = {}

        if node_map is not None:
            # Manual creation with provided node map
            self._identity = ""
            self._package_name = ""
            self._manifest_file_path = ""
            self._node_map = node_map.copy()
            return

        if identity is None:
            # Empty manifest
            self._identity = ""
            self._package_name = ""
            self._manifest_file_path = ""
            return

        # Load from resource identity
        self._identity = identity

        # Parse identity string formatted as <package_name>::<metadata_id>
        tokens = identity.split("::")
        if len(tokens) == 1:
            tokens.insert(0, "")

        if len(tokens) != 2:
            raise ResourceIdentityFormatError(
                f"Identity string '{identity}' has wrong format. Number of string tokens separated by '::' must be 2."
            )

        package_name = tokens[0]
        metadata_id = tokens[1]

        if not metadata_id:
            raise ResourceIdentityFormatError(
                f"Node manifest resource identity string '{identity}' has wrong format. " "Metadata ID cannot be empty."
            )

        # Find the resource in the ament index
        search_packages: set[str] = set()
        if package_name:
            search_packages.add(package_name)
        else:
            search_packages = set(ament_index_python.get_resources(RESOURCE_TYPE_NAME__NODE_MANIFEST))

        matching_count = 0
        self._package_name = ""
        self._manifest_file_path = ""

        for package in search_packages:
            try:
                content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__NODE_MANIFEST, package)

                for line in content.splitlines():
                    if not line:
                        continue

                    # Parse resource line format: metadata_id|manifest_path
                    parts = line.split("|")
                    if len(parts) != 2:
                        raise ResourceError(
                            f"Invalid node manifest resource file (Package: '{package}'). Invalid line: {line}."
                        )

                    found_metadata_id = parts[0]
                    manifest_path = parts[1]

                    # Check if this matches our search criteria
                    if found_metadata_id != metadata_id:
                        continue

                    matching_count += 1
                    self._package_name = package
                    self._manifest_file_path = os.path.join(base_path, manifest_path)

                    # Load the manifest data
                    if os.path.isfile(self._manifest_file_path):
                        with open(self._manifest_file_path, "r") as f:
                            manifest_data = yaml.safe_load(f)
                            if not isinstance(manifest_data, dict):
                                raise ResourceError(
                                    f"Node manifest file '{self._manifest_file_path}' is not a valid YAML mapping."
                                )

                            # Store raw dictionaries
                            for node_name, node_data in manifest_data.items():
                                if isinstance(node_data, dict):
                                    self._node_map[node_name] = node_data.copy()
                                else:
                                    raise ResourceError(
                                        f"Invalid node data for '{node_name}' in manifest '{self._manifest_file_path}'"
                                    )
                    else:
                        raise ResourceError(f"Node manifest file '{self._manifest_file_path}' does not exist.")

            except Exception as e:
                raise ResourceError(f"Error processing package {package}: {e}")

        if matching_count == 0:
            raise ResourceError(f"No node manifest with identity '{identity}' was registered.")
        if matching_count > 1:
            raise ResourceError(f"Resource identity '{identity}' is ambiguous. You must be more precise.")

    @property
    def identity(self) -> str:
        """Get the identity of this node manifest."""
        return self._identity

    @property
    def package_name(self) -> str:
        """Get the package name that registered this manifest."""
        return self._package_name

    @property
    def metadata_id(self) -> str:
        """Get the metadata ID of this manifest."""
        if "::" in self._identity:
            return self._identity.split("::")[-1]
        return self._identity

    @property
    def data(self) -> dict[str, dict]:
        """Get the manifest data as a dictionary of node option dictionaries."""
        return self._node_map.copy()

    @property
    def file_path(self) -> str:
        """Get the absolute path to the manifest file."""
        return self._manifest_file_path

    def get_node_names(self) -> list[str]:
        """Get all node names defined in this manifest."""
        return list(self._node_map.keys())

    def get_node_options(self, node_name: str) -> dict:
        """
        Get registration options for a specific node.

        Args:
            node_name: The name of the node to get options for.

        Returns:
            Dictionary containing node options.

        Raises:
            KeyError: If the node name is not found in the manifest.
        """
        if node_name not in self._node_map:
            raise KeyError(f"Node '{node_name}' not found in manifest '{self._identity}'")
        return self._node_map[node_name].copy()

    def has_node(self, node_name: str) -> bool:
        """Check if a node is defined in this manifest."""
        return node_name in self._node_map

    def add_node(self, node_name: str, options: dict) -> "NodeManifest":
        """
        Add registration options for a behavior tree node to the manifest.

        Args:
            node_name: Name of the behavior tree node.
            options: Dictionary of registration options to be used when loading the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            ValueError: If the registration options are invalid or node_name already exists.
        """
        if not isinstance(options, dict):
            raise ValueError(f"Options for node '{node_name}' must be a dictionary")
        if not options.get("class_name", "").strip():
            raise ValueError(f"Invalid registration options for node '{node_name}': class_name is required")
        if node_name in self._node_map:
            raise ValueError(f"Node '{node_name}' already exists in manifest")

        self._node_map[node_name] = options.copy()
        return self

    def remove_node(self, node_name: str) -> "NodeManifest":
        """
        Remove registration options for a behavior tree node.

        Args:
            node_name: Name of the behavior tree node.

        Returns:
            Modified node manifest (self).

        Raises:
            KeyError: If node_name doesn't exist.
        """
        if node_name not in self._node_map:
            raise KeyError(f"Node '{node_name}' not found in manifest")
        del self._node_map[node_name]
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
            ValueError: If other shares entries and replace is False.
        """
        for node_name, options in other._node_map.items():
            if node_name in self._node_map and not replace:
                raise ValueError(f"Node '{node_name}' already exists and replace=False")
            self._node_map[node_name] = options
        return self

    def size(self) -> int:
        """Get the number of behavior tree nodes this manifest holds registration options for."""
        return len(self._node_map)

    def empty(self) -> bool:
        """Determine whether any node registration options have been added to the manifest."""
        return len(self._node_map) == 0

    def to_dict(self) -> dict:
        """Convert the manifest to a dictionary representation suitable for YAML serialization."""
        result = {}
        for node_name, options in self._node_map.items():
            # Since options are already dictionaries, we can use them directly
            result[node_name] = dict(options)
        return result

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
                yaml.dump(self.to_dict(), f, default_flow_style=False)
        except Exception as e:
            raise IOError(f"Failed to write manifest to file '{file_path}': {e}")

    @staticmethod
    def from_files(file_paths: list[str]) -> "NodeManifest":
        """
        Create a node plugin manifest from multiple files. They are loaded in the given order.

        Args:
            file_paths: Paths to the manifest files.

        Returns:
            NodeManifest created by merging all files.

        Raises:
            ValueError: If merge fails for any file.
            IOError: If any file cannot be read.
        """
        manifest = NodeManifest()

        for file_path in file_paths:
            try:
                with open(file_path, "r") as f:
                    data = yaml.safe_load(f)
                    if not isinstance(data, dict):
                        raise ValueError(f"File '{file_path}' does not contain a valid YAML mapping")

                    file_manifest = NodeManifest()
                    for node_name, node_data in data.items():
                        if isinstance(node_data, dict):
                            file_manifest.add_node(node_name, node_data)
                        else:
                            raise ValueError(f"Invalid node data for '{node_name}' in file '{file_path}'")

                    manifest.merge(file_manifest, replace=True)

            except Exception as e:
                raise IOError(f"Failed to load manifest from file '{file_path}': {e}")

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
        return NodeManifest(identity)

    @staticmethod
    def find_by_metadata_id(metadata_id: str, package_name: str = "") -> "NodeManifest":
        """
        Find a node manifest by its metadata ID.

        Args:
            metadata_id: The metadata ID to search for.
            package_name: Optional package name to restrict search.

        Returns:
            NodeManifest object matching the criteria.
        """
        return NodeManifest(f"{package_name}::{metadata_id}")


@dataclass
class TreeResourceIdentity:
    package_name: str
    file_stem: str
    tree_name: str

    def __init__(self, identity: str = None):
        if identity is None:
            self.package_name = ""
            self.file_stem = ""
            self.tree_name = ""
            return

        # Parse identity string formatted as <package_name>::<file_stem>::<tree_name>
        tokens = identity.split("::")
        if len(tokens) == 1:
            tokens.insert(0, "")
            tokens.append("")

        if len(tokens) != 3:
            raise ResourceIdentityFormatError(
                f"Identity string '{identity}' has wrong format. Number of string tokens separated by '::' must be 3."
            )

        self.package_name = tokens[0]
        self.file_stem = tokens[1]
        self.tree_name = tokens[2]

        if not self.file_stem and not self.tree_name:
            raise ResourceIdentityFormatError(
                f"Behavior tree resource identity string '{identity}' has wrong format. "
                "It's not allowed to omit both <tree_file_stem> and <tree_name>."
            )

    def __str__(self) -> str:
        return f"{self.package_name}::{self.file_stem}::{self.tree_name}"

    def __eq__(self, other: "TreeResourceIdentity") -> bool:
        return str(self) == str(other)

    def __lt__(self, other: "TreeResourceIdentity") -> bool:
        return str(self) < str(other)

    def empty(self) -> bool:
        return not (self.package_name or self.file_stem or self.tree_name)


class TreeResource:
    def __init__(self, identity: TreeResourceIdentity | str):
        """Initialize a tree resource from an identity object or string."""
        if isinstance(identity, str):
            self._resource_identity = TreeResourceIdentity(identity)
        else:
            self._resource_identity = identity

        if self._resource_identity.empty():
            raise ResourceIdentityFormatError("Cannot create TreeResource with empty identity.")

        # Find the resource in the ament index
        search_packages: set[str] = set()
        if self._resource_identity.package_name:
            search_packages.add(self._resource_identity.package_name)
        else:
            search_packages = set(ament_index_python.get_resources(RESOURCE_TYPE_NAME__TREE))

        matching_count = 0
        self._package_name = ""
        self._tree_file_path = ""
        self._node_manifest_file_paths: list[str] = []
        self._doc_root_tree_name = ""

        for package in search_packages:
            try:
                content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__TREE, package)

                for line in content.splitlines():
                    if not line:
                        continue

                    # Parse resource line format: tree_file_stem|tree_names|file_path|manifest_paths
                    parts = line.split("|")
                    if len(parts) != 4:
                        raise ResourceError(
                            f"Invalid behavior tree resource file (Package: '{package}'). Invalid line: {line}."
                        )

                    found_tree_file_stem = parts[0]
                    found_tree_names = parts[1].split(";")

                    # Determine if resource matches the identity
                    if self._resource_identity.tree_name:
                        if self._resource_identity.file_stem:
                            if (
                                found_tree_file_stem != self._resource_identity.file_stem
                                or self._resource_identity.tree_name not in found_tree_names
                            ):
                                continue
                        else:
                            if self._resource_identity.tree_name not in found_tree_names:
                                continue
                    else:
                        if found_tree_file_stem != self._resource_identity.file_stem:
                            continue

                    matching_count += 1
                    self._package_name = package
                    self._tree_file_path = os.path.join(base_path, parts[2])

                    # Handle manifest paths
                    for path in parts[3].split(";"):
                        if os.path.isabs(path):
                            self._node_manifest_file_paths.append(path)
                        else:
                            self._node_manifest_file_paths.append(os.path.join(base_path, path))

            except Exception as e:
                print(f"Error processing package {package}: {e}")
                raise

        if matching_count == 0:
            raise ResourceError(f"No behavior tree file with identity '{self._resource_identity}' was registered.")
        if matching_count > 1:
            raise ResourceError(
                f"Resource identity '{self._resource_identity}' is ambiguous. You must be more precise."
            )

    @property
    def identity(self) -> TreeResourceIdentity:
        return self._resource_identity

    @property
    def content(self) -> str:
        """
        Writes the XML content of the behavior tree resource in a string an returns it.
        """
        with open(self._tree_file_path, "r") as f:
            xml_content = f.read()
        return xml_content

    @property
    def node_manifest(self) -> dict:
        """
        Concatenate and merge all node manifest YAML files into a single dictionary.
        Duplicate keys are not allowed and will raise a ValueError.
        Returns:
            dict: The merged dictionary containing all node manifests.
        Raises:
            FileNotFoundError: If a manifest file does not exist.
            ValueError: If a manifest is not a YAML mapping or contains duplicate keys.
        """
        merged = {}
        for manifest_path in self._node_manifest_file_paths:
            if not os.path.isfile(manifest_path):
                raise FileNotFoundError(f"Node manifest file not found: {manifest_path}")
            with open(manifest_path, "r") as f:
                data = yaml.safe_load(f)
                if not isinstance(data, dict):
                    raise ValueError(f"Node manifest {manifest_path} is not a YAML mapping")
                for k in data:
                    if k in merged:
                        raise ValueError(f"Duplicate node name '{k}' found in node manifest: {manifest_path}")
                merged.update(data)
        return merged

    @staticmethod
    def select_by_tree_name(tree_name: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using a specific behavior tree name.

        Args:
            tree_name (str): The name of the behavior tree.
            package_name (str, optional): The name of the package to search in. Default: Search in all installed packages.

        Returns:
            TreeResource: The matching behavior tree resource.
        """
        return TreeResource(f"{package_name}::::{tree_name}")

    @staticmethod
    def select_by_file_stem(file_stem: str, package_name: str = "") -> "TreeResource":
        """
        Find an installed behavior tree resource using a specific behavior tree XML file stem.

        Args:
            file_stem (str): The file stem of the behavior tree XML file.
            package_name (str, optional): The name of the package to search in. Default: Search in all installed packages.

        Returns:
            TreeResource: The matching behavior tree resource.
        """
        return TreeResource(f"{package_name}::{file_stem}::")

    def get_package_name(self) -> str:
        """
        Get the name of the package this resource was registered by.

        Returns:
            str: The package name.
        """
        return self._package_name

    def get_file_stem(self) -> str:
        """
        Get the file stem of the XML file containing the tree document.

        Returns:
            str: The file stem.
        """
        return Path(self._tree_file_path).stem

    def create_identity(self, tree_name: str = "") -> TreeResourceIdentity:
        """
        Create a valid tree resource identity representing this resource.

        Args:
            tree_name (str, optional): The tree name to use in the identity. Default: Refer to the main tree.

        Returns:
            TreeResourceIdentity: The constructed identity.
        """
        identity = TreeResourceIdentity()
        identity.package_name = self._package_name
        identity.file_stem = self.get_file_stem()
        identity.tree_name = tree_name
        return identity


def get_all_behavior_tree_resources(exclude_packages: set[str] = None) -> list[TreeResource]:
    """
    Retrieves all available/installed behavior tree resources.

    This function scans all packages that have registered behavior tree resources,
    parses their resource files, and constructs a list of `TreeResource` objects
    representing each available behavior tree.

    Returns:
        List[TreeResource]: A sorted list of `TreeResource` objects, each corresponding
        to a discovered behavior tree resource.

    Raises:
        ValueError: If a resource file contains an invalid line.
    """
    trees = []

    # Get all packages that have registered behavior tree resources
    packages = get_packages_with_resource_type(RESOURCE_TYPE_NAME__TREE, exclude_packages)
    for package in packages:
        # Get the content of the resource file
        content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__TREE, package)

        for line in content.splitlines():
            if not line:
                continue

            parts = line.split("|")
            if len(parts) != 4:
                raise ValueError(f"Invalid behavior tree resource file (Package: '{package}'). Invalid line: {line}.")

            file_stem = parts[0]
            tree_names = parts[1].split(";")

            # Create TreeResource for each tree name
            for tree_name in tree_names:
                if tree_name:  # Skip empty tree names
                    tree = TreeResource(f"{package}::{file_stem}::{tree_name}")
                    trees.append(tree)

    return sorted(trees, key=lambda x: str(x.identity))


def get_all_behavior_tree_node_plugins(exclude_packages: set[str] = None) -> list[str]:
    """
    Get all behavior tree node plugin names.

    This is a convenience function that finds all plugins with the behavior tree node
    base class type.

    Args:
        exclude_packages: Packages to exclude when searching for plugins.

    Returns:
        List of all behavior tree node plugin names.

    Raises:
        ResourceError: If failed to find or parse plugin manifest files.
    """
    return get_plugin_names_with_base_type(BASE_CLASS_TYPE__BEHAVIOR_TREE_NODE, exclude_packages)


def get_all_behavior_tree_node_manifest_resources(exclude_packages: set[str] = None) -> list[NodeManifest]:
    """
    Get all available node manifest resources as NodeManifest objects.

    Args:
        exclude_packages: Packages to exclude when searching for manifests.

    Returns:
        List of all NodeManifest objects.

    Raises:
        ResourceError: If failed to find or parse manifest files.
    """
    manifests = []
    packages = get_packages_with_resource_type(RESOURCE_TYPE_NAME__NODE_MANIFEST, exclude_packages)

    for package in packages:
        try:
            content, base_path = ament_index_python.get_resource(RESOURCE_TYPE_NAME__NODE_MANIFEST, package)

            for line in content.splitlines():
                if not line:
                    continue

                # Parse resource line format: metadata_id|manifest_path
                parts = line.split("|")
                if len(parts) != 2:
                    raise ResourceError(
                        f"Invalid node manifest resource file (Package: '{package}'). Invalid line: {line}."
                    )

                metadata_id = parts[0]
                manifests.append(NodeManifest(f"{package}::{metadata_id}"))

        except Exception as e:
            raise ResourceError(f"Error reading node manifest from package '{package}': {e}")

    return sorted(manifests, key=lambda x: x.identity)
