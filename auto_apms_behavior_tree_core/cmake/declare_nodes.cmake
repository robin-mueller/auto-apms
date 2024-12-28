# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Add plugins that register behavior tree nodes at runtime to the
# package's resources.
#
# This macro must be called to make behavior tree node plugins available
# at runtime and configure their registration with the behavior tree
# factory. Optionally, a corresponding node model header is generated.
# This header facilitates integrating the specified nodes when building
# behavior trees using the TreeDocument API.
#
# :param target: Shared library target implementing the behavior tree
#   nodes declared under ARGN.
# :type target: string
# :param ARGN: The unique names of node classes being declared with this
#   macro call and exported by the shared library target.
# :type ARGN: list of strings
# :param NODE_MANIFEST: Path or identifier of the node manifest YAML file.
# :type NODE_MANIFEST: string
# :param MODEL_HEADER_TARGET: If specified, generate a C++ header that
#   defines model classes for all behavior tree nodes given in ARGN and
#   add it to the includes of this shared library target.
# :type MODEL_HEADER_TARGET: string
#
# @public
#
macro(auto_apms_behavior_tree_declare_nodes target)

    # Parse arguments
    set(options "")
    set(oneValueArgs MODEL_HEADER_TARGET)
    set(multiValueArgs NODE_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    auto_apms_util_register_plugins(
        ${target}
        "auto_apms_behavior_tree::core::NodeRegistrationInterface"
        ${ARGS_UNPARSED_ARGUMENTS}
        FACTORY_TEMPLATE_CLASS "auto_apms_behavior_tree::core::NodeRegistrationTemplate"
    )

    # Append build information of the specified node plugins (<class_name>@<library_path>).
    # Make sure to do before calling generating the node metadata (Otherwise build info would be unavailable).
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
        list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO "${_class_name}@$<TARGET_FILE:${target}>")
    endforeach()

    # Automatically create node metadata if any manifest files are provided
    if(NOT "${ARGS_NODE_MANIFEST}" STREQUAL "")
        auto_apms_behavior_tree_generate_node_metadata("${target}" ${ARGS_NODE_MANIFEST} MODEL_HEADER_TARGET "${ARGS_MODEL_HEADER_TARGET}")
    endif()

endmacro()
