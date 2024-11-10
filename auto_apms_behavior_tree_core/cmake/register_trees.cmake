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

macro(auto_apms_behavior_tree_register_trees xml_file_path)

    # Check if behavior tree file exists
    get_filename_component(_tree_abs_path__source "${xml_file_path}" REALPATH)
    if(NOT EXISTS "${_tree_abs_path__source}")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_trees(): Behavior tree file ${_tree_abs_path__source} does not exist"
        )
    endif()

    get_filename_component(_tree_file_name "${_tree_abs_path__source}" NAME)
    get_filename_component(_tree_file_stem "${_tree_abs_path__source}" NAME_WE)

    # Verify no duplicates in tree file names
    if("${_tree_file_name}" IN_LIST _tree_file_names)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_trees(): A behavior tree file with name '${_tree_file_name}' was already registered"
        )
    endif()
    list(APPEND _tree_file_names "${_tree_file_name}")

    # Collect all available behavior tree IDs
    file(READ "${_tree_abs_path__source}" _tree_file_content)
    string(REGEX MATCHALL "<BehaviorTree ID=\"[A-Za-z0-9_]+\">" _matches "${_tree_file_content}")
    if(_matches STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_trees(): Behavior tree file ${_tree_abs_path__source} doesn't specify any valid behavior trees"
        )
    endif()
    set(_file_tree_names "")
    foreach(_match ${_matches})
        string(REGEX MATCH "<BehaviorTree ID=\"([A-Za-z0-9_]+)\">" _ "${_match}")
        set(_tree_name ${CMAKE_MATCH_1})
        # Verify no duplicates in tree IDs
        if("${_tree_name}" IN_LIST _all_tree_names)
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_register_trees(): Behavior tree with name '${_tree_name}' was already registered"
            )
        endif()
        list(APPEND _all_tree_names "${_tree_name}")
        list(APPEND _file_tree_names "${_tree_name}")
    endforeach()

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs NODE_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_tree_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__TREE}")
    set(_node_manifest_rel_path__install "")
    set(_node_manifest_file_name "node_model_${_tree_file_stem}.xml")

    if(NOT ${ARGS_NODE_MANIFEST} STREQUAL "")
        auto_apms_behavior_tree_create_node_metadata("${_tree_file_stem}"
            ${ARGS_NODE_MANIFEST}
            MANIFEST_FILE_NAME "${_node_manifest_file_name}"
        )
        set(_node_manifest_rel_path__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/${_node_manifest_file_name}")
    endif()

    # Install behavior tree
    install(
        FILES "${_tree_abs_path__source}"
        DESTINATION "${_tree_rel_dir__install}")

    # Fill meta info
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}${_tree_file_stem}|${_tree_rel_dir__install}/${_tree_file_name}|${_node_manifest_rel_path__install}|${_file_tree_names}\n")

endmacro()
