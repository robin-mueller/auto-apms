macro(px4_behavior_register_plugin_config)
    # Register plugin configuration files to make them available during runtime
    install(
        FILES ${ARGV}
        DESTINATION "${_PX4_BEHAVIOR_RESOURCES_DIR}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}"
    )

    # Generate node model for plugin configuration
    foreach(_config_yaml_source_path ${ARGV})
        # Set variables for cli command
        get_filename_component(_config_yaml_source_stem "${_config_yaml_source_path}" NAME_WE)
        string(REPLACE " " "_" _config_yaml_source_stem "${_config_yaml_source_stem}")
        string(TOLOWER "${_config_yaml_source_stem}" _config_yaml_source_stem)
        set(_model_build_path "${PROJECT_BINARY_DIR}/${_config_yaml_source_stem}_model.xml")

        # Add custom command and target for model generation
        add_custom_command(OUTPUT "${_model_build_path}"
            COMMAND "${_PX4_BEHAVIOR_EXECUTABLES_INSTALL_DIR}/generate_bt_node_model"
                "${_config_yaml_source_path}" # Absolute path of the config source file
                "${_model_build_path}" # Absolute directory to write the model definition file to
                ${_PX4_BEHAVIOR_BT_NODE_PLUGINS_BUILD_INFO} # Library paths of the plugins built in this package. Their install locations are not available at build time
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS "${_config_yaml_source_path}" ${_node_plugin_targets}
            COMMENT "Generate node model definition according to plugin configuration ${_config_yaml_source_path}.")
        add_custom_target(_target_generate_${_config_yaml_source_stem}_model ALL
            DEPENDS "${_model_build_path}")

        # Install the generated file to share
        install(
            FILES "${_model_build_path}"
            DESTINATION "${_PX4_BEHAVIOR_RESOURCES_DIR}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}")
    endforeach()
endmacro()