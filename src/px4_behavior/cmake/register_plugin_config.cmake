macro(px4_behavior_register_plugin_config)
    # Register plugin configuration files to make them available during runtime
    install(
        FILES ${ARGV}
        DESTINATION "${_PX4_BEHAVIOR_RESOURCES_DIR}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}"
    )

    ### Generate node model for plugin configuration

    # Get all plugin targets of this build
    px4_behavior_get_all_bt_plugin_targets(_all_bt_plugin_targets)
    foreach(tgt ${_all_bt_plugin_targets})
        list(APPEND _extra_plugin_paths "$<TARGET_FILE:${tgt}>")
    endforeach()
    

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
                ${_extra_plugin_paths} # Additional plugin paths to be preferred over the installation.
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS "${_config_yaml_source_path}" ${_all_bt_plugin_targets}
            COMMENT "Generate node model definition according to plugin configuration ${_config_yaml_source_path}.")
        add_custom_target(_target_generate_${_config_yaml_source_stem}_model ALL
            DEPENDS "${_model_build_path}")

        # Install the generated file to share
        install(
            FILES "${_model_build_path}"
            DESTINATION "${_PX4_BEHAVIOR_RESOURCES_DIR}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}")
    endforeach()
endmacro()