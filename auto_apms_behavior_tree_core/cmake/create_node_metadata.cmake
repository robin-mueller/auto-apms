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

macro(auto_apms_behavior_tree_create_node_metadata suffix)

    # Parse arguments
    set(options "")
    set(oneValueArgs MANIFEST_FILE_NAME MODEL_FILE_NAME)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(${ARGS_UNPARSED_ARGUMENTS} STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_create_nodes_model(): You didn't provide any node manifest yaml files."
        )
    endif()

    # Default names
    set(_node_manifest_file_name "node_manifest_${suffix}.yaml")
    set(_node_model_file_name "node_model_${suffix}.xml")

    if(NOT ${ARGS_MANIFEST_FILE_NAME} STREQUAL "")
        get_filename_component(_node_manifest_file_stem "${ARGS_MANIFEST_FILE_NAME}" NAME_WE)
        set(_node_manifest_file_name "${_node_manifest_file_stem}.yaml")
    endif()

    if(NOT ${ARGS_MODEL_FILE_NAME} STREQUAL "")
        get_filename_component(_node_model_file_stem "${ARGS_MODEL_FILE_NAME}" NAME_WE)
        set(_node_model_file_name "${_node_model_file_stem}.xml")
    endif()

    list(REMOVE_DUPLICATES ARGS_UNPARSED_ARGUMENTS) # Disregard duplicates
    set(_node_manifest_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}")
    set(_node_model_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}")

    # Create a valid suffix for the custom target
    string(REPLACE " " "_" _node_model_custom_target_suffix "${suffix}")
    string(TOLOWER "${_node_model_custom_target_suffix}" _node_model_custom_target_suffix)

    # Create the complete manifest for model generation.
    # Simultaneously, define a variable containing the library paths and generator expressions for node plugin dependencies.
    file(MAKE_DIRECTORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}")
    set(_node_manifest_abs_path__build "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_node_manifest_file_name}")
    set(_node_manifest_rel_path__install "${_node_manifest_rel_dir__install}/${_node_manifest_file_name}")
    execute_process(
        COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MANIFEST_CMD}"
            "${ARGS_UNPARSED_ARGUMENTS}" # Paths of the manifest source files

            # General build information for node plugins compiled by this package.
            # Generator expressions cannot be evaluated yet since execute_process is handled at configuration time.
            # However, we don't require the correct library paths yet.
            "${_AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO}"

            "${PROJECT_NAME}"  # Name of the package that builds the behavior tree model
            "${_node_manifest_abs_path__build}"  # File to write the behavior tree node plugin manifest to
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE _node_library_paths
        RESULT_VARIABLE _return_code
        ERROR_VARIABLE _error
    )
    if(_return_code EQUAL 1)
        message(
            FATAL_ERROR
            "Failed to create node plugin manifest '${suffix}' parsing [${ARGS_UNPARSED_ARGUMENTS}].\n${_error}"
        )
    endif()

    # Use the above created manifest for generating a node model
    set(_node_model_abs_path__build "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_node_model_file_name}")
    add_custom_command(OUTPUT "${_node_model_abs_path__build}"
        COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_CMD}"
            "\"${_node_manifest_abs_path__build}\"" # Path to the processed node plugin manifest

            # Exhaustive list of libraries to be loaded by ClassLoader.
            # create_node_manifest previously collected all library paths that are required to successfully load the nodes specified in the manifest file.
            # We pass this variable to the command to specifiy which libraries to load and to add target-level dependencies for those targets mentioned in any $<TARGET_FILE:tgt> generator expressions.
            # Therefore, we configure the compilation so that any shared libraries created by the package invoking the macro are built before being used here.
            # If we wouldn't do this, there would be an error saying 'there is no rule to make target ...'.
            # Additionally, this variable needs to be passed to DEPENDS to create a file-level dependency to the shared library files which makes sure that the command is executed when they are recompiled.
            "\"${_node_library_paths}\""

            "\"${_node_model_abs_path__build}\"" # File to write the behavior tree node model to
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        DEPENDS ${ARGS_NODE_MANIFEST} ${_node_library_paths}
        COMMENT "Generate behavior tree node model for tree file '${_tree_file_stem}' with libraries [${_node_library_paths}].")
    add_custom_target(_target_create_node_model__${_node_model_custom_target_suffix} ALL
        DEPENDS "${_node_model_abs_path__build}")

    # Install the generated node plugin manifest file
    install(
        FILES "${_node_manifest_abs_path__build}"
        DESTINATION "${_node_manifest_rel_dir__install}"
    )

    # Install the generated node model file
    install(
        FILES "${_node_model_abs_path__build}"
        DESTINATION "${_node_model_rel_dir__install}"
    )

endmacro()