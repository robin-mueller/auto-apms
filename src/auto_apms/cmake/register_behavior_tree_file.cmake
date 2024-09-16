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
            "auto_apms_register_behavior_tree_file(): Behavior tree file ${tree_filepath} has already been registered"
        )
    endif()
    list(APPEND _tree_filepaths "${tree_filepath}")

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs PLUGIN_CONFIGS TREE_IDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    get_filename_component(_tree_file_name "${tree_filepath_abs}" NAME)
    get_filename_component(_tree_file_stem "${tree_filepath_abs}" NAME_WE)

    set(_rel_tree_install_dir "${_AUTO_APMS_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME}")
    set(_rel_plugin_config_install_paths "")

    if(NOT ${ARGS_PLUGIN_CONFIGS} STREQUAL "")

        set(_rel_plugin_config_install_dir "${_AUTO_APMS_RESOURCES_DIR_RELATIVE}/${_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_DIR_NAME}")
        set(_rel_node_model_install_dir "${_rel_tree_install_dir}")
        foreach(_plugin_config_filepath ${ARGS_PLUGIN_CONFIGS})

            # Check if plugin file exists
            get_filename_component(_plugin_config_filepath_abs "${_plugin_config_filepath}" REALPATH)
            if(NOT EXISTS "${_plugin_config_filepath_abs}")
                message(
                    FATAL_ERROR
                    "auto_apms_register_behavior_tree_file(): Plugin file ${_plugin_config_filepath_abs} does not exist"
                )
            endif()

            get_filename_component(_plugin_config_file_name "${_plugin_config_filepath_abs}" NAME)

            # Verify no duplicates in config files
            set(_plugin_config_filepath_install "${_rel_plugin_config_install_dir}/${_plugin_config_file_name}")
            if("${_plugin_config_filepath_install}" IN_LIST _rel_plugin_config_install_paths)
                message(
                    FATAL_ERROR
                    "auto_apms_register_behavior_tree_file(): Plugin config file path ${_plugin_config_filepath} was provided multiple times"
                )
            endif()
            list(APPEND _rel_plugin_config_install_paths "${_plugin_config_filepath_install}")
        endforeach()

        # Generate plugin node model for according to the plugin configuration

        # Make sure the custom target name is valid
        string(REPLACE " " "_" tgt_suffix "${_tree_file_stem}")
        string(TOLOWER "${tgt_suffix}" tgt_suffix)

        # Add custom command and target for model generation
        set(_model_build_path "${PROJECT_BINARY_DIR}/${_tree_file_stem}_node_model.xml")
        add_custom_command(OUTPUT "${_model_build_path}"
            COMMAND "${_AUTO_APMS_EXECUTABLES_INSTALL_DIR}/generate_bt_node_model"
                "\"${ARGS_PLUGIN_CONFIGS}\"" # Paths of the config source files
                "\"${_model_build_path}\"" # Directory to write the behavior tree model file to
                "\"${PROJECT_NAME}\""  # Name of the package that builds the behavior tree model
                "\"${_AUTO_APMS_BT_NODE_PLUGINS__BUILD_INFO}\"" # Library paths of the plugins built in this package. Their install locations are not available at build time
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS ${ARGS_PLUGIN_CONFIGS} # TODO: Let CMake parse the config file and also add depencencies from other installs
            COMMENT "Generate node model definition with config paths ${ARGS_PLUGIN_CONFIGS}.")
        add_custom_target(_target_generate_bt_node_model__${tgt_suffix} ALL
            DEPENDS "${_model_build_path}")

        # Install the generated node model file
        install(
            FILES "${_model_build_path}"
            DESTINATION "${_rel_node_model_install_dir}")


        # Install plugin configuration files to make them available during runtime
        install(
            FILES ${ARGS_PLUGIN_CONFIGS}
            DESTINATION "${_rel_plugin_config_install_dir}"
        )

    endif()

    # Install behavior tree
    install(
        FILES "${tree_filepath_abs}"
        DESTINATION "${_rel_tree_install_dir}")

    # Fill meta info
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE}${_tree_file_stem}|${_rel_tree_install_dir}/${_tree_file_name}|${_rel_plugin_config_install_paths}|${ARGS_TREE_IDS}|\n")

endmacro()
