macro(px4_behavior_register_plugin_config)
    set(options "")
    set(oneValueArgs DIRECTORY)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(DEFINED ARGS_UNPARSED_ARGUMENTS)
        install(
            FILES ${ARGS_UNPARSED_ARGUMENTS}
            DESTINATION "share/${_PX4_BEHAVIOR_RESOURCES_DIR_NAME}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}")
    endif()
    if(NOT ${ARGS_DIRECTORY} STREQUAL "")
        install(
            DIRECTORY ${ARGS_DIRECTORY}/
            DESTINATION "share/${_PX4_BEHAVIOR_RESOURCES_DIR_NAME}/${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}")
    endif()
endmacro()

macro(px4_behavior_register_behavior_tree)
    set(options "")
    set(oneValueArgs DIRECTORY)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(NOT ${ARGS_UNPARSED_ARGUMENTS})
        install(
            FILES ${ARGS_UNPARSED_ARGUMENTS}
            DESTINATION "share/${_PX4_BEHAVIOR_RESOURCES_DIR_NAME}/${_PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME}")
    endif()
    if(NOT ${ARGS_DIRECTORY} STREQUAL "")
        install(
            DIRECTORY ${ARGS_DIRECTORY}/
            DESTINATION "share/${_PX4_BEHAVIOR_RESOURCES_DIR_NAME}/${_PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME}")
    endif()
endmacro()
