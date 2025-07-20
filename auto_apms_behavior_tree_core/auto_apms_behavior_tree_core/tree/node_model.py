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

import xml.etree.ElementTree as ET

from enum import Enum
from typing import NamedTuple


class NodeType(Enum):
    UNDEFINED = 0
    ACTION = 1
    CONDITION = 2
    CONTROL = 3
    DECORATOR = 4
    SUBTREE = 5

    @staticmethod
    def from_string(s: str) -> "NodeType":
        if s == "Action":
            return NodeType.ACTION
        if s == "Condition":
            return NodeType.CONDITION
        if s == "Control":
            return NodeType.CONTROL
        if s == "Decorator":
            return NodeType.DECORATOR
        if s == "SubTree":
            return NodeType.SUBTREE
        return NodeType.UNDEFINED


class NodePortDirection(Enum):
    INPUT = 0
    OUTPUT = 1
    INOUT = 2


class NodePortInfo(NamedTuple):
    port_name: str
    port_type: str
    port_default: str
    port_has_default: bool
    port_description: str
    port_direction: NodePortDirection


class NodeModel(NamedTuple):
    type: NodeType
    port_infos: list[NodePortInfo]


class NodeModelMap(dict[str, NodeModel]):
    """
    Represents a behavior tree node model as a custom dictionary mapping the
    registration names to the respective NodePortInfo objects.
    """

    def __init__(self, xml_path: str):
        super().__init__()
        self._parse_model_file(xml_path)

    def _parse_model_file(self, xml_path: str) -> None:
        """Parse the XML model file and populate the dictionary."""
        try:
            tree = ET.parse(xml_path)
            root = tree.getroot()

            # Find the TreeNodesModel element
            tree_nodes_model = root.find("TreeNodesModel")
            if tree_nodes_model is None:
                raise ValueError("TreeNodesModel not found in XML file")

            # Parse each node type (Action, Condition, Control, Decorator, SubTree)
            for node_element in tree_nodes_model:
                # Parse all ports for this node
                port_infos = []
                for port_element in node_element:
                    port_info = self._parse_port(port_element)
                    if port_info:
                        port_infos.append(port_info)
                key = node_element.get("ID", "")
                if key:
                    self[key] = NodeModel(type=NodeType.from_string(node_element.tag), port_infos=port_infos)

        except ET.ParseError as e:
            raise ValueError(f"Failed to parse XML file: {e}")
        except FileNotFoundError:
            raise FileNotFoundError(f"Model file not found: {xml_path}")

    def _parse_port(self, port_element) -> NodePortInfo:
        """Parse a single port element and return NodePortInfo."""
        port_name = port_element.get("name", "")
        port_type = port_element.get("type", "")
        port_default = port_element.get("default", "")
        port_has_default = "default" in port_element.attrib
        port_description = port_element.text.strip() if port_element.text else ""

        # Determine port direction from tag name
        tag_name = port_element.tag.lower()
        if tag_name == "input_port":
            port_direction = NodePortDirection.INPUT
        elif tag_name == "output_port":
            port_direction = NodePortDirection.OUTPUT
        elif tag_name == "inout_port":
            port_direction = NodePortDirection.INOUT
        else:
            # Default to INPUT for unknown port types
            port_direction = NodePortDirection.INPUT

        return NodePortInfo(
            port_name=port_name,
            port_type=port_type,
            port_default=port_default,
            port_has_default=port_has_default,
            port_description=port_description,
            port_direction=port_direction,
        )
