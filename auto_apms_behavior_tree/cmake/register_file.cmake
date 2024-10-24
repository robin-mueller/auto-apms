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

macro(auto_apms_behavior_tree_register_file tree_filepath)

    # Check if behavior tree file exists
    get_filename_component(_tree_abs_path__source "${tree_filepath}" REALPATH)
    if(NOT EXISTS "${_tree_abs_path__source}")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_file(): Behavior tree file ${_tree_abs_path__source} does not exist"
        )
    endif()

    get_filename_component(_tree_file_name "${_tree_abs_path__source}" NAME)
    get_filename_component(_tree_file_stem "${_tree_abs_path__source}" NAME_WE)

    # Verify no duplicates in tree file names
    if("${_tree_file_name}" IN_LIST _tree_file_names)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_file(): A behavior tree file with name '${_tree_file_name}' was already registered"
        )
    endif()
    list(APPEND _tree_file_names "${_tree_file_name}")

    # Collect all available behavior tree IDs
    file(READ "${_tree_abs_path__source}" _tree_file_content)
    string(REGEX MATCHALL "<BehaviorTree ID=\"[A-Za-z0-9_]+\">" _matches "${_tree_file_content}")
    if(_matches STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_file(): Behavior tree file ${_tree_abs_path__source} doesn't specify any valid behavior trees"
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
                "auto_apms_behavior_tree_register_file(): Behavior tree with name '${_tree_name}' was already registered"
            )
        endif()
        list(APPEND _all_tree_names "${_tree_name}")
        list(APPEND _file_tree_names "${_tree_name}")
    endforeach()

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs NODE_PLUGIN_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(_tree_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__TREE}")
    set(_node_plugin_manifest_rel_path__install "")

    if(NOT ${ARGS_NODE_PLUGIN_MANIFEST} STREQUAL "")
        file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}")
        list(REMOVE_DUPLICATES ARGS_NODE_PLUGIN_MANIFEST) # Disregard duplicates
        set(_node_plugin_manifest_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__NODE}")
        set(_node_plugin_manifest_created_file_name "${_tree_file_stem}_node_plugin_manifest.yaml")
        set(_node_model_rel_dir__install "${_tree_rel_dir__install}")

        ##############################################################################
        #### Generate node model according to the behavior tree's plugin configuration

        # Create a valid suffix for the custom target
        string(REPLACE " " "_" _node_mode_custom_target_suffix "${_tree_file_stem}")
        string(TOLOWER "${_node_mode_custom_target_suffix}" _node_mode_custom_target_suffix)

        # Create the complete manifest for model generation.
        # However, this command is mainly relevant for defining a variable containing the library paths and generator expressions
        # for direct node plugin dependencies of the registered behavior tree
        set(_node_plugin_manifest_abs_path__build "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${_node_plugin_manifest_created_file_name}")
        execute_process(
            COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_INTERNAL_CLI_INSTALL_DIR}/create_node_plugin_manifest"
                "${ARGS_NODE_PLUGIN_MANIFEST}" # Paths of the manifest source files

                # General build information for node plugins compiled by this package.
                # Generator expressions cannot be evaluated yet since execute_process is handled at configuration time.
                # However, we don't require the correct library paths yet.
                "${_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_BUILD_INFO}"

                "${PROJECT_NAME}"  # Name of the package that builds the behavior tree model
                "${_node_plugin_manifest_abs_path__build}"  # File to write the behavior tree node plugin manifest to
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _node_plugin_library_paths
            RESULT_VARIABLE _return_code)
        if(_return_code EQUAL 1)
            message(
                FATAL_ERROR
                "Failed to create node plugin manifest for '${_tree_file_stem}' parsing [${ARGS_NODE_PLUGIN_MANIFEST}]"
            )
        endif()
        # Create a command to evaluate the generator expressions inculded in the manifest file after the CMake configuration stage.
        # NOTE: The file isn't fully processed right after the invocation of the file(GENERATE ...) macro.
        file(GENERATE OUTPUT "${_node_plugin_manifest_abs_path__build}" INPUT "${_node_plugin_manifest_abs_path__build}")
        set(_node_plugin_manifest_rel_path__install "${_node_plugin_manifest_rel_dir__install}/${_node_plugin_manifest_created_file_name}")

        # Use the above created manifest for generating the node model
        set(_node_model_abs_path__build "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${_tree_file_stem}_node_model.xml")
        add_custom_command(OUTPUT "${_node_model_abs_path__build}"
            COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_INTERNAL_CLI_INSTALL_DIR}/generate_node_model"
                "\"${_node_plugin_manifest_abs_path__build}\"" # Path to the processed node plugin manifest
                "\"${_node_model_abs_path__build}\"" # File to write the behavior tree node model to

                # IMPORTANT: Since we have the manifest, passing this variable to COMMAND isn't strictly necessary,
                # but because this adds target-level dependencies to the custom command, it is required.
                # All targets in _node_plugin_library_paths (generator expression or not) ar added as dependencies to the command.
                # If this would be omitted, there would be an error saying 'there is no rule to make target ...'.
                # Additionally, this variable needs to be passed to DEPENDS as well to create a file-level dependency
                # to the shared libraries (Ensuring the command is executed when they are recompiled).
                "\"${_node_plugin_library_paths}\""
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS ${ARGS_NODE_PLUGIN_MANIFEST} ${_node_plugin_library_paths}
            COMMENT "Generate behavior tree node model for tree file '${_tree_file_stem}' with libraries [${_node_plugin_library_paths}].")
        add_custom_target(_target_generate_node_model__${_node_mode_custom_target_suffix} ALL
            DEPENDS "${_node_model_abs_path__build}")

        ##############################################################################

        # Install the generated node model file
        install(
            FILES "${_node_model_abs_path__build}"
            DESTINATION "${_node_model_rel_dir__install}")


        # Install the generated node plugin manifest file
        install(
            FILES "${_node_plugin_manifest_abs_path__build}"
            DESTINATION "${_node_plugin_manifest_rel_dir__install}"
        )

    endif()

    # Install behavior tree
    install(
        FILES "${_tree_abs_path__source}"
        DESTINATION "${_tree_rel_dir__install}")

    # Fill meta info
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}${_tree_file_stem}|${_tree_rel_dir__install}/${_tree_file_name}|${_node_plugin_manifest_rel_path__install}|${_file_tree_names}|\n")

endmacro()
