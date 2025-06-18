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
from typing import List, Set

AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE = "auto_apms_behavior_tree_core__tree"


class ResourceIdentityFormatError(Exception):
    pass


class ResourceError(Exception):
    pass


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
        search_packages: Set[str] = set()
        if self._resource_identity.package_name:
            search_packages.add(self._resource_identity.package_name)
        else:
            search_packages = set(
                ament_index_python.get_resources(AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE)
            )

        matching_count = 0
        self._package_name = ""
        self._tree_file_path = ""
        self._node_manifest_file_paths: List[str] = []
        self._doc_root_tree_name = ""

        for package in search_packages:
            try:
                content, base_path = ament_index_python.get_resource(
                    AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE, package
                )

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


def get_all_behavior_trees() -> List[TreeResource]:
    """
    Retrieves all available behavior tree resources registered in the system.

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
    packages = ament_index_python.get_resources(AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE)
    for package in packages:
        # Get the content of the resource file
        content, base_path = ament_index_python.get_resource(
            AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__TREE, package
        )

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
