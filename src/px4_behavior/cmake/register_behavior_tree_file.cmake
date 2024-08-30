macro(px4_behavior_register_behavior_tree_file tree_filepath)
    
    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs PLUGIN_CONFIGS TREE_IDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    get_filename_component(_tree_file_name "${tree_filepath}" NAME)
    get_filename_component(_tree_file_stem "${tree_filepath}" NAME_WE)

    set(_rel_tree_install_dir "${_PX4_BEHAVIOR_RESOURCES_DIR_RELATIVE}/${_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_DIR_NAME}")
    set(_rel_plugin_config_install_paths "")

    if (NOT ${ARGS_PLUGIN_CONFIGS} STREQUAL "")

        set(_rel_plugin_config_install_dir "${_PX4_BEHAVIOR_RESOURCES_DIR_RELATIVE}/${_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_DIR_NAME}")
        set(_rel_node_model_install_dir "${_rel_tree_install_dir}")
        foreach(_plugin_config_filepath ${ARGS_PLUGIN_CONFIGS})
            get_filename_component(_plugin_config_file_name "${_plugin_config_filepath}" NAME)
            list(APPEND _rel_plugin_config_install_paths "${_rel_plugin_config_install_dir}/${_plugin_config_file_name}")
        endforeach()

        # Generate plugin node model for according to the plugin configuration

        # Make sure the custom target name is valid
        string(REPLACE " " "_" tgt_suffix "${_tree_file_stem}")
        string(TOLOWER "${tgt_suffix}" tgt_suffix)

        # Add custom command and target for model generation
        set(_model_build_path "${PROJECT_BINARY_DIR}/${_tree_file_stem}_node_model.xml")
        add_custom_command(OUTPUT "${_model_build_path}"
            COMMAND "${_PX4_BEHAVIOR_EXECUTABLES_INSTALL_DIR}/generate_bt_node_model"
                "\"${ARGS_PLUGIN_CONFIGS}\"" # Absolute path of the config source file
                "\"${_model_build_path}\"" # Absolute directory to write the model definition file to
                "\"${_PX4_BEHAVIOR_BT_NODE_PLUGINS__BUILD_INFO}\"" # Library paths of the plugins built in this package. Their install locations are not available at build time
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS ${ARGS_PLUGIN_CONFIGS} ${_PX4_BEHAVIOR_BT_NODE_PLUGINS__TARGETS} # TODO: Let CMake parse the config file and add depencencies specifically
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
        FILES "${tree_filepath}"
        DESTINATION "${_rel_tree_install_dir}")

    # Fill meta info
    set(_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_FILE "${_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_FILE}${_tree_file_stem}|${_rel_tree_install_dir}/${_tree_file_name}|${_rel_plugin_config_install_paths}|${ARGS_TREE_IDS}|\n")
    
endmacro()
