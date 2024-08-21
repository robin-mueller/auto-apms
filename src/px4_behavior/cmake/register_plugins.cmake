# Macro that registers a library target for a behavior tree node plugin
macro(px4_behavior_register_plugins plugin_lib_target)
    if(NOT TARGET ${plugin_lib_target})
        message(
        FATAL_ERROR
        "px4_behavior_register_plugins() first argument '${plugin_lib_target}' is not a target")
    endif()
    
    # Check target type
    get_target_property(_target_type ${plugin_lib_target} TYPE)
    if(NOT _target_type STREQUAL "SHARED_LIBRARY")
        message(
        FATAL_ERROR
        "px4_behavior_register_plugins() first argument '${plugin_lib_target}' is not a shared library target")
    endif()
    
    cmake_parse_arguments(ARGS "" "" "" ${ARGN})
    set(_unique_names)
    foreach(_arg ${ARGS_UNPARSED_ARGUMENTS})
        if(_arg IN_LIST _unique_names)
            message(
            FATAL_ERROR
            "px4_behavior_register_plugins() the plugin names must be unique (multiple '${_arg}')")
        endif()
        list(APPEND _unique_names "${_arg}")

        # Append to the variable that holds the resource information of the behavior tree node plugins
        if(WIN32)
            set(_path "bin")
        else()
            set(_path "lib")
        endif()

        # Fill meta info
        list(APPEND _PX4_BEHAVIOR_BT_NODE_PLUGINS__TARGETS ${plugin_lib_target})
        list(APPEND _PX4_BEHAVIOR_BT_NODE_PLUGINS__BUILD_INFO "${_arg}|$<TARGET_FILE:${plugin_lib_target}>")
        set(_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_FILE "${_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_FILE}${_arg}|${_path}/$<TARGET_FILE_NAME:${plugin_lib_target}>\n")
    endforeach()

endmacro()
