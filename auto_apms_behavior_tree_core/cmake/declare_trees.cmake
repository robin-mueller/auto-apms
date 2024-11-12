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

macro(auto_apms_behavior_tree_declare_trees xml_file_path)

    # Check if behavior tree file exists
    get_filename_component(_tree_abs_path__source "${xml_file_path}" REALPATH)
    if(NOT EXISTS "${_tree_abs_path__source}")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_declare_trees(): Behavior tree file ${_tree_abs_path__source} does not exist"
        )
    endif()

    get_filename_component(_tree_file_name "${_tree_abs_path__source}" NAME)
    get_filename_component(_tree_file_stem "${_tree_abs_path__source}" NAME_WE)

    # Verify no duplicates in tree file names
    if("${_tree_file_name}" IN_LIST _tree_file_names)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_declare_trees(): A behavior tree file with name '${_tree_file_name}' was already registered"
        )
    endif()
    list(APPEND _tree_file_names "${_tree_file_name}")

    # Collect all available behavior tree IDs
    file(READ "${_tree_abs_path__source}" _tree_file_content)
    string(REGEX MATCHALL "<BehaviorTree ID=\"[A-Za-z0-9_]+\">" _matches "${_tree_file_content}")
    if(_matches STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_declare_trees(): Behavior tree file ${_tree_abs_path__source} doesn't specify any valid behavior trees"
        )
    endif()
    set(_tree_file_tree_names "")
    foreach(_match ${_matches})
        string(REGEX MATCH "<BehaviorTree ID=\"([A-Za-z0-9_]+)\">" _ "${_match}")
        set(_tree_name ${CMAKE_MATCH_1})
        # Verify no duplicate tree IDs
        if("${_tree_name}" IN_LIST _all_tree_names)
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_declare_trees(): Behavior tree with name '${_tree_name}' was already registered"
            )
        endif()
        list(APPEND _all_tree_names "${_tree_name}")
        list(APPEND _tree_file_tree_names "${_tree_name}")
    endforeach()

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs NODE_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_tree_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__TREE}")
    set(_node_manifest_rel_paths__install "") # Empty string if no manifest is given
    set(_node_manifest_file_name "node_manifest_${_tree_file_stem}.xml")

    if(NOT ${ARGS_NODE_MANIFEST} STREQUAL "")

        # Collect existing manifest info
        set(_existing_node_metadata_ids "")
        set(_existing_node_manifest_abs_paths__build "")
        foreach(_tuple ${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_BUILD_INFO})
            # Split each tuple by '@'
            string(REPLACE "@" ";" _tuple_parts "${_tuple}")

            list(GET _tuple_parts 0 _id)
            list(GET _tuple_parts 1 _path)
            list(APPEND _existing_node_metadata_ids "${_id}")
            list(APPEND _existing_node_manifest_abs_paths__build "${_path}")
        endforeach()

        # Change the input to the auto_apms_behavior_tree_generate_node_metadata macro call so that we use all existing metadata
        set(_generate_node_metadata_inputs ${ARGS_NODE_MANIFEST})
        set(_matching_existing_metadata_count 0)
        foreach(_var ${ARGS_NODE_MANIFEST})
            # Check if metadata for _var has already been generated
            list(FIND _existing_node_metadata_ids "${_var}" _index)
            if(NOT _index EQUAL -1)
            # If matching metadata is existing, remove the corresponding item in the inputs and use the existing node manifest file path instead
            list(GET _existing_node_manifest_abs_paths__build "${_index}" _path)
            list(REMOVE_ITEM _generate_node_metadata_inputs "${_var}")
            list(APPEND _generate_node_metadata_inputs "${_path}")
            math(EXPR _matching_existing_metadata_count "${_matching_existing_metadata_count} + 1")
            endif()
        endforeach()

        # Is there any existing metadata registered under the given metadata ID(s)?
        if(_matching_existing_metadata_count EQUAL 0)
            # We cannot use any existing node manifests and must generate everything
            auto_apms_behavior_tree_generate_node_metadata("${_tree_file_stem}" ${ARGS_NODE_MANIFEST})

            # Sticking to the default manifest install file path for the resource info
            set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_tree_file_stem}.yaml")
        else()
            list(LENGTH _generate_node_metadata_inputs _manifest_args_length)
            if(_manifest_args_length EQUAL _matching_existing_metadata_count)
                # We don't have to generate anything because all required node manifests have already been generated under the provided ID and there will be corresponding install paths. We just forward those.
                foreach(_build_path ${_existing_node_manifest_abs_paths__build})
                    get_filename_component(_file_name "${_build_path}" NAME)
                    list(APPEND _node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/${_file_name}")
                endforeach()
            else()
                # There are some already registered node manifests (looked up using the provided metadata ID(s)) but we also have to generate additional metadata. So we look up the existing node manifest files and use them to generate the metadata under a new ID.
                auto_apms_behavior_tree_generate_node_metadata("${_tree_file_stem}" ${_generate_node_metadata_inputs})

                # Sticking to the default manifest install file path for the resource info
                set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_tree_file_stem}.yaml")
            endif()
        endif()
    endif()

    # Install behavior tree
    install(
        FILES "${_tree_abs_path__source}"
        DESTINATION "${_tree_rel_dir__install}")

    # Fill resource info
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}${_tree_file_stem}|${_tree_rel_dir__install}/${_tree_file_name}|${_node_manifest_rel_paths__install}|${_tree_file_tree_names}\n")

endmacro()
