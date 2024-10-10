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

macro(auto_apms_register_behavior_tree_file tree_filepath)

    # Check if behavior tree file exists
    get_filename_component(tree_filepath_abs "${tree_filepath}" REALPATH)
    if(NOT EXISTS "${tree_filepath_abs}")
        message(
            FATAL_ERROR
            "auto_apms_register_behavior_tree_file(): Behavior tree file ${tree_filepath_abs} does not exist"
        )
    endif()

    # Verify no duplicates in tree files
    if("${tree_filepath}" IN_LIST _tree_filepaths)
        message(
            FATAL_ERROR
            "auto_apms_register_behavior_tree_file(): Behavior tree file '${tree_filepath}' was already registered"
        )
    endif()
    list(APPEND _tree_filepaths "${tree_filepath}")

    # Verify BTCPP_format exists and is 4
    file(READ "${tree_filepath}" _tree_file_content)
    string(REGEX MATCHALL "BTCPP_format=\"4\"" _matches "${_tree_file_content}")
    list(LENGTH _matches _root_btcpp_format_matches_length)
    if(NOT _root_btcpp_format_matches_length EQUAL 1)
        message(
            FATAL_ERROR
            "auto_apms_register_behavior_tree_file(): Behavior tree file '${tree_filepath}' has wrong XML format"
        )
    endif()

    # Collect all available behavior tree IDs
    string(REGEX MATCHALL "<BehaviorTree ID=\"[A-Za-z0-9_]+\">" _matches "${_tree_file_content}")
    if(_matches STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_register_behavior_tree_file(): Behavior tree file '${tree_filepath}' doesn't specify any valid behavior trees"
        )
    endif()
    set(_file_tree_ids "")
    foreach(_match ${_matches})
        string(REGEX MATCH "<BehaviorTree ID=\"([A-Za-z0-9_]+)\">" _ "${_match}")
        set(_tree_id ${CMAKE_MATCH_1})

        # Verify no duplicates in tree IDs
        if("${_tree_id}" IN_LIST _tree_ids)
            message(
                FATAL_ERROR
                "auto_apms_register_behavior_tree_file(): Behavior tree ID '${_tree_id}' was already registered"
            )
        endif()
        list(APPEND _tree_ids "${_tree_id}")
        list(APPEND _file_tree_ids "${_tree_id}")
    endforeach()

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs NODE_PLUGIN_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    get_filename_component(_tree_file_name "${tree_filepath_abs}" NAME)
    get_filename_component(_tree_file_stem "${tree_filepath_abs}" NAME_WE)

    set(_rel_tree_install_dir "${_AUTO_APMS_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__TREE}")
    set(_rel_plugin_config_install_paths "")

    if(NOT ${ARGS_NODE_PLUGIN_MANIFEST} STREQUAL "")

        set(_rel_plugin_config_install_dir "${_AUTO_APMS_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__NODE}")
        set(_rel_node_model_install_dir "${_rel_tree_install_dir}")
        foreach(_node_plugin_manifest_path ${ARGS_NODE_PLUGIN_MANIFEST})

            # Check if plugin file exists
            get_filename_component(_node_plugin_manifest_path_abs "${_node_plugin_manifest_path}" REALPATH)
            if(NOT EXISTS "${_node_plugin_manifest_path_abs}")
                message(
                    FATAL_ERROR
                    "auto_apms_register_behavior_tree_file(): Node plugin manifest file ${_node_plugin_manifest_path_abs} does not exist"
                )
            endif()

            get_filename_component(_plugin_config_file_name "${_node_plugin_manifest_path_abs}" NAME)

            # Verify no duplicates in config files
            set(_node_plugin_manifest_path_install "${_rel_plugin_config_install_dir}/${_plugin_config_file_name}")
            if("${_node_plugin_manifest_path_install}" IN_LIST _rel_plugin_config_install_paths)
                message(
                    FATAL_ERROR
                    "auto_apms_register_behavior_tree_file(): Node plugin manifest file ${_node_plugin_manifest_path} was provided multiple times"
                )
            endif()
            list(APPEND _rel_plugin_config_install_paths "${_node_plugin_manifest_path_install}")
        endforeach()

        ##############################################################################
        #### Generate node model according to the behavior tree's plugin configuration

        # Make sure the custom target name is valid
        string(REPLACE " " "_" tgt_suffix "${_tree_file_stem}")
        string(TOLOWER "${tgt_suffix}" tgt_suffix)

        # Create the complete manifest for model generation.
        # However, this command is mainly relevant for defining a variable containing the library paths and generator expressions
        # for direct node plugin dependencies of the registered behavior tree
        set(_node_plugin_manifest__build_path "${PROJECT_BINARY_DIR}/${_tree_file_stem}_node_plugin_manifest__build.yaml")
        execute_process(
            COMMAND "${_AUTO_APMS_INTERNAL_CLI_INSTALL_DIR}/create_node_plugin_manifest"
                "${ARGS_NODE_PLUGIN_MANIFEST}" # Paths of the manifest source files

                # General build information for node plugins compiled by this package.
                # Generator expressions cannot be evaluated yet since execute_process is handled at configuration time.
                # However, we don't require the correct library paths yet.
                "${_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_BUILD_INFO}"

                "${PROJECT_NAME}"  # Name of the package that builds the behavior tree model
                "${_node_plugin_manifest__build_path}"  # File to write the behavior tree node load manifest to
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _node_plugin_library_paths
            RESULT_VARIABLE _return_code)
        if(_return_code EQUAL 1)
            message(
                FATAL_ERROR
                "Failed to write node plugin manifest for '${_tree_file_stem}' parsing [${ARGS_NODE_PLUGIN_MANIFEST}]"
            )
        endif()
        # Create a command to evaluate the generator expressions inculded in the manifest file after the CMake configuration stage.
        # NOTE: The file isn't fully processed right after the invocation of the file(GENERATE ...) macro.
        file(GENERATE OUTPUT "${_node_plugin_manifest__build_path}" INPUT "${_node_plugin_manifest__build_path}")

        # Use the above created manifest for generating the node model
        set(_model_build_path "${PROJECT_BINARY_DIR}/${_tree_file_stem}_node_model.xml")
        add_custom_command(OUTPUT "${_model_build_path}"
            COMMAND "${_AUTO_APMS_INTERNAL_CLI_INSTALL_DIR}/generate_node_model"
                "\"${_node_plugin_manifest__build_path}\"" # Path to the complete node plugin manifest

                # Libraries to be loaded.
                # IMPORTANT: Since we have the manifest, passing this to COMMAND doesn't seem necessary,
                # but it is, because this adds a target-level dependency to targets configured by this package
                # which may be present in _node_plugin_library_paths as generator expressions.
                # Otherwise, there would be an error saying 'there is no rule to make target'.
                # Additionally, this variable needs to be passed to DEPENDS as well to create a file-level dependency
                # to the shared libraries (Ensuring the command is executed when they are recompiled).
                "\"${_node_plugin_library_paths}\""

                "\"${_model_build_path}\"" # File to write the behavior tree node model to
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS ${ARGS_NODE_PLUGIN_MANIFEST} ${_node_plugin_library_paths}
            COMMENT "Generate behavior tree node model for tree file '${_tree_file_stem}' with libraries [${_node_plugin_library_paths}].")
        add_custom_target(_target_generate_node_model__${tgt_suffix} ALL
            DEPENDS "${_model_build_path}")

        ##############################################################################

        # Install the generated node model file
        install(
            FILES "${_model_build_path}"
            DESTINATION "${_rel_node_model_install_dir}")


        # Install plugin configuration files to make them available during runtime
        install(
            FILES ${ARGS_NODE_PLUGIN_MANIFEST}
            DESTINATION "${_rel_plugin_config_install_dir}"
        )

    endif()

    # Install behavior tree
    install(
        FILES "${tree_filepath_abs}"
        DESTINATION "${_rel_tree_install_dir}")

    # Fill meta info
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}${_tree_file_stem}|${_rel_tree_install_dir}/${_tree_file_name}|${_rel_plugin_config_install_paths}|${_file_tree_ids}|\n")

endmacro()
