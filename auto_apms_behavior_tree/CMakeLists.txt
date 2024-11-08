cmake_minimum_required(VERSION 3.12)
project(auto_apms_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(auto_apms_util)
find_package(auto_apms_interfaces REQUIRED)

# Configure and include BehaviorTree.CPP library
include(FetchContent)
FetchContent_Declare(behaviortree_cpp
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG 4.6.2
    GIT_SHALLOW true
    GIT_PROGRESS true
)
set(BTCPP_BUILD_TOOLS       OFF CACHE BOOL "")
set(BTCPP_EXAMPLES          OFF CACHE BOOL "")
set(BTCPP_UNIT_TESTS        OFF CACHE BOOL "")
set(BTCPP_GROOT_INTERFACE   ON  CACHE BOOL "")
set(BTCPP_SQLITE_LOGGING    OFF CACHE BOOL "")
FetchContent_MakeAvailable(behaviortree_cpp)

#
# ------------ Configuration --------------
#

option(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING "Enable debug logging severity" ON)
if(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING)
    add_compile_definitions(_AUTO_APMS_DEBUG_LOGGING)
endif()

set(_AUTO_APMS_BEHAVIOR_TREE_INTERNAL_CLI_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/internal")
set(_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE "auto_apms_behavior_tree")

# Behavior tree executor parameter names
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_CREATOR "tree_creator_name")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE "tick_rate")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT "groot2_port")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER "state_change_logger")

# Node manifest parameter names
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS "class_name")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT "port")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT "wait_timeout")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT "request_timeout")

# Resources
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__TREE "behavior_trees")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE "auto_apms_behavior_tree__tree")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE "auto_apms_behavior_tree__node")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__CREATOR "auto_apms_behavior_tree__creator")

#
# ------------ Library --------------
#

# Generate parameter headers employing https://github.com/PickNikRobotics/generate_parameter_library
configure_file("src/executor/executor_params.yaml.in" "${CMAKE_CURRENT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/executor_params.yaml")
set(_temp ${CMAKE_CURRENT_SOURCE_DIR})  # WORKAROUND: temporarily change CMAKE_CURRENT_SOURCE_DIR since the function doesn't allow to load the parameter definition file from a different directory
set(CMAKE_CURRENT_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}")
generate_parameter_library(executor_params "executor_params.yaml")
set(CMAKE_CURRENT_SOURCE_DIR ${_temp})

add_library(auto_apms_behavior_tree SHARED
    "src/node/node_registration_params.cpp"
    "src/node/node_manifest.cpp"
    "src/node/ros_node_context.cpp"
    "src/creator/tree_builder.cpp"
    "src/creator/tree_creator_base.cpp"
    "src/executor/state_observer.cpp"
    "src/executor/executor.cpp"
    "src/executor/executor_server.cpp"
    "src/resource/tree_resource.cpp"
    "src/resource/node_registration_class_loader.cpp"
    "src/resource/tree_creator_class_loader.cpp"
    "src/util/bt_types.cpp"
)
target_include_directories(auto_apms_behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(auto_apms_behavior_tree PRIVATE
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__CREATOR="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__CREATOR}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_CREATOR="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_CREATOR}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER}"
)
ament_target_dependencies(auto_apms_behavior_tree
    rclcpp
    rclcpp_action
    rclcpp_components
    TinyXML2
    auto_apms_util
    auto_apms_interfaces
)
target_link_libraries(auto_apms_behavior_tree
    behaviortree_cpp
    executor_params
)

# Install the version header to the autogenerated headers dir also used by rosidl generators
ament_generate_version_header(auto_apms_behavior_tree HEADER_PATH "${PROJECT_NAME}/version.hpp")

# Default tree executor server node. Available as a typical ROS node component and a standalone executable.
rclcpp_components_register_node(auto_apms_behavior_tree
    PLUGIN "auto_apms_behavior_tree::TreeExecutorServer"
    EXECUTABLE "tree_executor"
)

#
# ------------ Behavior Tree Nodes --------------
#

add_library(tree_nodes SHARED
    "src/node/impl/action/throw_exception.cpp"
    "src/node/impl/action/load_bt.cpp"
    "src/node/impl/action/launch_executor.cpp"
    "src/node/impl/action/halt_executor.cpp"
    "src/node/impl/action/pause_executor.cpp"
    "src/node/impl/action/resume_executor.cpp"
)
target_link_libraries(tree_nodes auto_apms_behavior_tree)

# In the original package, we need to manually include this before being able to call the macro.
# However, it is automatically available for all downstream packages.
include("cmake/register_nodes.cmake")

auto_apms_behavior_tree_register_nodes(tree_nodes
    "auto_apms_behavior_tree::ThrowException"
    "auto_apms_behavior_tree::LoadBehaviorTreeAction"
    "auto_apms_behavior_tree::LaunchExecutorAction"
    "auto_apms_behavior_tree::HaltExecutorAction"
    "auto_apms_behavior_tree::PauseExecutorAction"
    "auto_apms_behavior_tree::ResumeExecutorAction"
)

#
# ------------ Tree Builders --------------
#

add_library(tree_creators SHARED
    "src/creator/impl/resource_tree_creator.cpp"
)
target_link_libraries(tree_creators auto_apms_behavior_tree)

# In the original package, we need to manually include this before being able to call the macro.
# However, it is automatically available for all downstream packages.
include("cmake/register_creators.cmake")

auto_apms_behavior_tree_register_creators(tree_creators
    "auto_apms_behavior_tree::ResourceTreeCreator"
)

#
# ------------ Utility Executables --------------
#

# Merge multiple node manifest files to a single one and collect the shared library paths for the required nodes
add_executable(create_node_manifest "src/cli/internal/create_node_manifest.cpp")
target_link_libraries(create_node_manifest auto_apms_behavior_tree)

# Generate the node model according to a node manifest
add_executable(generate_node_model "src/cli/internal/generate_node_model.cpp")
target_link_libraries(generate_node_model auto_apms_behavior_tree)

# Run a behavior tree registered with the ament resource index
add_executable(run_tree "src/cli/run_tree.cpp")
target_link_libraries(run_tree auto_apms_behavior_tree)

# Run a single behavior tree node registered with the ament resource index
add_executable(run_tree_node "src/cli/run_tree_node.cpp")
target_link_libraries(run_tree_node auto_apms_behavior_tree)

#
# ------------ Packaging --------------
#

###### Export libraries ######
install(
    TARGETS
    behaviortree_cpp
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
    TinyXML2
    auto_apms_util
    auto_apms_interfaces
)

###### Plugins ######
install(
    TARGETS
    tree_nodes
    tree_creators
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

###### Publicly available cli tools ######
install(
    TARGETS
    run_tree
    run_tree_node
    DESTINATION "lib/${PROJECT_NAME}"
)

###### Internal executables ######
install(
    TARGETS
    create_node_manifest
    generate_node_model
    DESTINATION "${_AUTO_APMS_BEHAVIOR_TREE_INTERNAL_CLI_INSTALL_DIR}"
)

install(
    DIRECTORY "include/${PROJECT_NAME}"
    DESTINATION "include"
)

install(
    DIRECTORY "cmake"
    DESTINATION "share/${PROJECT_NAME}"
)

# For downstream packages this is registered as an extension to ament_package,
# but here we need to include this script manually to install the resource marker files
include("cmake/register_resources_hook.cmake")

ament_package(CONFIG_EXTRAS "auto_apms_behavior_tree-extras.cmake.in")
