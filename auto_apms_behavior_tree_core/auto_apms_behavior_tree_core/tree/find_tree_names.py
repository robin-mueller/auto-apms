#!/usr/bin/env python3

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

import re
import sys

PATTERN = r'<BehaviorTree\s+[^>]*\bID="([A-Za-z0-9_]+)"'


def find_tree_names_in_string(xml_str: str) -> list[str]:
    """Finds all behavior tree names in an XML string.

    :param xml_str: XML data.
    :type xml_str: str
    :return: A list of all behavior tree names found in the XML string.
    :rtype: list[str]
    """
    return re.findall(PATTERN, xml_str)


def find_tree_names_in_file(xml_file):
    """Finds all behavior tree names in an XML file.

    :param xml_file: Path to XML file.
    :type xml_file: str
    :return: A list of all behavior tree names found in the XML file.
    :rtype: list[str]
    """
    with open(xml_file, "r") as file:
        xml_content = file.read()

    return re.findall(PATTERN, xml_content)


# Used in CMakeLists.txt
if __name__ == "__main__":
    print(*find_tree_names_in_file(sys.argv[1]), sep=";")
