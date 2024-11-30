cmake_minimum_required(VERSION 3.12)
project(auto_apms_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)

#
# ------------ Configuration --------------
#

option(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING "Enable debug logging severity" ON)
if(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING)
    add_compile_definitions(_AUTO_APMS_DEBUG_LOGGING)
endif()

set(_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE "${_AUTO_APMS_UTIL__THIS_PACKAGE_BUILD_DIR_ABSOLUTE}")

# Behavior tree executor defaults
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME "tree_executor")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX "/start")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX "/cmd")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS "allow_other_build_handlers")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD "allow_dynamic_blackboard")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS "allow_dynamic_scripting_enums")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE "node_exclude_packages")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER "build_handler_exclude_packages")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER "build_handler")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE "tick_rate")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT "groot2_port")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER "state_change_logger")

# Builtin behavior tree node names
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_LOGGER "Logger")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_THROW_EXCEPTION "Error")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR "StartExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_TERMINATE_EXECUTOR "TerminateExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_HALT_EXECUTOR "HaltExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_PAUSE_EXECUTOR "PauseExecutor")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_RESUME_EXECUTOR "ResumeExecutor")

set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER "SetParameter")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL "SetParameterBool")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT "SetParameterInt")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE "SetParameterDouble")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING "SetParameterString")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BYTE_VEC "SetParameterByteVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL_VEC "SetParameterBoolVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT_VEC "SetParameterIntVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE_VEC "SetParameterDoubleVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING_VEC "SetParameterStringVec")

set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER "GetParameter")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL "GetParameterBool")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT "GetParameterInt")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE "GetParameterDouble")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING "GetParameterString")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BYTE_VEC "GetParameterByteVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL_VEC "GetParameterBoolVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT_VEC "GetParameterIntVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE_VEC "GetParameterDoubleVec")
set(_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING_VEC "GetParameterStringVec")

#
# ------------ Library --------------
#

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
configure_file("config/executor_params.yaml.in" "${_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE}/executor_params.yaml")
set(_temp ${CMAKE_CURRENT_SOURCE_DIR})  # WORKAROUND: temporarily change CMAKE_CURRENT_SOURCE_DIR since the function doesn't allow to load the parameter definition file from a different directory
set(CMAKE_CURRENT_SOURCE_DIR "${_AUTO_APMS_BEHAVIOR_TREE__BUILD_DIR_ABSOLUTE}")
generate_parameter_library(executor_params "executor_params.yaml")
set(CMAKE_CURRENT_SOURCE_DIR ${_temp})

add_library(auto_apms_behavior_tree SHARED
    "src/build_handler/build_handler.cpp"
    "src/build_handler/build_handler_loader.cpp"
    "src/executor/state_observer.cpp"
    "src/executor/executor_base.cpp"
    "src/executor/executor_node.cpp"
    "src/util/conversion.cpp"
)
target_include_directories(auto_apms_behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(auto_apms_behavior_tree PUBLIC
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER}"

    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_LOGGER="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_LOGGER}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_THROW_EXCEPTION="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_THROW_EXCEPTION}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_START_EXECUTOR}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_TERMINATE_EXECUTOR="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_TERMINATE_EXECUTOR}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_HALT_EXECUTOR="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_HALT_EXECUTOR}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_PAUSE_EXECUTOR="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_PAUSE_EXECUTOR}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_RESUME_EXECUTOR="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_RESUME_EXECUTOR}"

    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BYTE_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BYTE_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_BOOL_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_INT_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_DOUBLE_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_SET_PARAMETER_STRING_VEC}"

    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BYTE_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BYTE_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_BOOL_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_INT_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_DOUBLE_VEC}"
    _AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING_VEC="${_AUTO_APMS_BEHAVIOR_TREE__BUILTIN_NODE_NAME_GET_PARAMETER_STRING_VEC}"
)
target_link_libraries(auto_apms_behavior_tree
    executor_params
)
ament_target_dependencies(auto_apms_behavior_tree
    rclcpp
    rclcpp_action
    tinyxml2_vendor
    auto_apms_util
    auto_apms_interfaces
    auto_apms_behavior_tree_core
)

# Install the version header to the autogenerated headers dir also used by rosidl generators
ament_generate_version_header(auto_apms_behavior_tree HEADER_PATH "${PROJECT_NAME}/version.hpp")

# Default tree executor server node. Available as a typical ROS node component and a standalone executable.
# We create a separate shared library for that since it's best practice when using class_loader::ClassLoader
# for loading plugins as you can read here https://wiki.ros.org/class_loader under section 2.8.
add_library(executor_node_components SHARED
    "src/executor/executor_node_components.cpp"
)
target_link_libraries(executor_node_components
    auto_apms_behavior_tree
)
ament_target_dependencies(executor_node_components
    rclcpp_components
)
rclcpp_components_register_node(executor_node_components
    PLUGIN "auto_apms_behavior_tree::TreeExecutorNode"
    EXECUTABLE "tree_executor"
)
rclcpp_components_register_nodes(executor_node_components
    "auto_apms_behavior_tree::NoUndeclaredParamsExecutorNode"
    "auto_apms_behavior_tree::OnlyScriptingEnumParamsExecutorNode"
    "auto_apms_behavior_tree::OnlyBlackboardParamsExecutorNode"
    "auto_apms_behavior_tree::OnlyInitialScriptingEnumParamsExecutorNode"
    "auto_apms_behavior_tree::OnlyInitialBlackboardParamsExecutorNode"
)

#
# ------------ Builtin Behavior Tree Node Plugins --------------
#

add_library(builtin_nodes SHARED
    "src/node/action/logger.cpp"
    "src/node/action/throw_exception.cpp"
    "src/node/action/start_executor.cpp"
    "src/node/action/command_executor.cpp"
    "src/node/service/set_parameter.cpp"
    "src/node/service/get_parameter.cpp"

    "src/util/conversion.cpp"
)
# Make conversion util available for builtin nodes too, but don't include anything that's not also been added to the sources
target_include_directories(builtin_nodes PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(builtin_nodes
    rclcpp
    rcl_interfaces
    auto_apms_interfaces
    auto_apms_behavior_tree_core
)
auto_apms_behavior_tree_declare_nodes(builtin_nodes
    "auto_apms_behavior_tree::Logger"
    "auto_apms_behavior_tree::ThrowException"
    "auto_apms_behavior_tree::StartExecutor"
    "auto_apms_behavior_tree::TerminateExecutor"
    "auto_apms_behavior_tree::HaltExecutor"
    "auto_apms_behavior_tree::PauseExecutor"
    "auto_apms_behavior_tree::ResumeExecutor"

    "auto_apms_behavior_tree::SetParameter"
    "auto_apms_behavior_tree::SetParameterBool"
    "auto_apms_behavior_tree::SetParameterInt"
    "auto_apms_behavior_tree::SetParameterDouble"
    "auto_apms_behavior_tree::SetParameterString"
    "auto_apms_behavior_tree::SetParameterByteVec"
    "auto_apms_behavior_tree::SetParameterBoolVec"
    "auto_apms_behavior_tree::SetParameterIntVec"
    "auto_apms_behavior_tree::SetParameterDoubleVec"
    "auto_apms_behavior_tree::SetParameterStringVec"

    "auto_apms_behavior_tree::GetParameter"
    "auto_apms_behavior_tree::GetParameterBool"
    "auto_apms_behavior_tree::GetParameterInt"
    "auto_apms_behavior_tree::GetParameterDouble"
    "auto_apms_behavior_tree::GetParameterString"
    "auto_apms_behavior_tree::GetParameterByteVec"
    "auto_apms_behavior_tree::GetParameterBoolVec"
    "auto_apms_behavior_tree::GetParameterIntVec"
    "auto_apms_behavior_tree::GetParameterDoubleVec"
    "auto_apms_behavior_tree::GetParameterStringVec"
    NODE_MANIFEST
    "config/builtin_nodes.yaml.in"
)

#
# ------------ Tree build handlers --------------
#

add_library(builtin_build_handlers SHARED
    "src/build_handler/impl/resource_build_handler.cpp"
)
target_link_libraries(builtin_build_handlers auto_apms_behavior_tree)

# In the original package, we need to manually include this before being able to call the macro.
# However, it is automatically available for all downstream packages.
include("cmake/declare_build_handlers.cmake")
auto_apms_behavior_tree_declare_build_handlers(builtin_build_handlers
    "auto_apms_behavior_tree::TreeResourceBuildHandler"
)

#
# ------------ Utility Executables --------------
#

# Run a behavior tree registered with the ament resource index
add_executable(run_tree "src/cli/run_tree.cpp")
target_link_libraries(run_tree auto_apms_behavior_tree)

# Run a single behavior tree node registered with the ament resource index
add_executable(run_tree_node "src/cli/run_tree_node.cpp")
target_link_libraries(run_tree_node auto_apms_behavior_tree)

# Create a new behavior tree file with optionally a node model already loaded
add_executable(new_tree "src/cli/new_tree.cpp")
target_link_libraries(new_tree auto_apms_behavior_tree)

#
# ------------ Testing --------------
#

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # To include the builtin_nodes manifest, we can pass the target name
    # that was previously associated with the generation of the respective metadata.
    auto_apms_behavior_tree_declare_trees(
        "test/trees/infinite_w_bb_flags.xml"
        NODE_MANIFEST
        builtin_nodes
    )

    auto_apms_behavior_tree_declare_trees(
        "test/trees/builtin_nodes_test.xml"
        NODE_MANIFEST
        builtin_nodes
    )

    # ament_add_gtest(${PROJECT_NAME}_unit_tests
    #     "test/unit/main.cpp"
    # )
    # ament_target_dependencies(${PROJECT_NAME}_unit_tests
    #     auto_apms_interfaces
    # )
    # target_link_libraries(${PROJECT_NAME}_unit_tests
    #     auto_apms_util
    # )
endif()

#
# ------------ Packaging --------------
#

###### Export libraries ######
install(
    TARGETS
    executor_params
    auto_apms_behavior_tree
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    rclcpp_action
    tinyxml2_vendor
    auto_apms_util
    auto_apms_interfaces
    auto_apms_behavior_tree_core
    generate_parameter_library # Any parameter library targets depend on generate_parameter_library's dependencies
)

###### Plugins ######
install(
    TARGETS
    builtin_nodes
    builtin_build_handlers
    executor_node_components
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    TARGETS
    run_tree
    run_tree_node
    new_tree
    DESTINATION "lib/${PROJECT_NAME}"
)

install(
    DIRECTORY "include/${PROJECT_NAME}"
    DESTINATION "include"
)

install(
    DIRECTORY "cmake"
    DESTINATION "share/${PROJECT_NAME}"
)

ament_package(CONFIG_EXTRAS "auto_apms_behavior_tree-extras.cmake.in")
