cmake_minimum_required(VERSION 3.12)
project(auto_apms_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(auto_apms_core)
find_package(auto_apms_interfaces REQUIRED)

option(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING "Enable debug logging severity" ON)
if(AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING)
    add_compile_definitions(_AUTO_APMS_DEBUG_LOGGING)
endif()

set(_AUTO_APMS_BEHAVIOR_TREE_INTERNAL_CLI_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/internal")

# Behavior tree executor parameter names
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_BUILDER "tree_builder_name")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE "tick_rate")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT "groot2_port")
set(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER "state_change_logger")

# Node manifest parameter names
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS "class_name")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PACKAGE "package")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_LIBRARY "library")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT "port")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT "wait_timeout")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT "request_timeout")

set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__TREE "behavior_trees")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE "auto_apms_behavior_tree__tree")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE "auto_apms_behavior_tree__node")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__BUILDER "auto_apms_behavior_tree__builder")
set(_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE "auto_apms_behavior_tree")

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
    "src/node/node_manifest.cpp"
    "src/node/ros_node_context.cpp"
    "src/builder/tree_builder.cpp"
    "src/builder/tree_builder_base.cpp"
    "src/executor/state_observer.cpp"
    "src/executor/executor.cpp"
    "src/executor/executor_server.cpp"
    "src/resource/tree_resource.cpp"
    "src/resource/node_class_loader.cpp"
    "src/resource/tree_builder_class_loader.cpp"
)
target_include_directories(auto_apms_behavior_tree PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(auto_apms_behavior_tree PRIVATE
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__BUILDER="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__BUILDER}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_CLASS}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PACKAGE="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PACKAGE}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_LIBRARY="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_LIBRARY}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_PORT}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_WAIT_TIMEOUT}"
    _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT="${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_PARAM_REQUEST_TIMEOUT}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_BUILDER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_BUILDER}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT}"
    _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER="${_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER}"
)
ament_target_dependencies(auto_apms_behavior_tree
    rclcpp
    rclcpp_action
    rclcpp_components
    pluginlib
    behaviortree_cpp
    yaml_cpp_vendor
    TinyXML2
    auto_apms_core
    auto_apms_interfaces
)
target_link_libraries(auto_apms_behavior_tree
    executor_params
)

# Install the version header to the autogenerated headers dir also used by rosidl generators
ament_generate_version_header(auto_apms_behavior_tree HEADER_PATH "${PROJECT_NAME}/version.hpp")

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

# In the original package, we need to manually include this before being able to call the register_nodes macro.
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

add_library(tree_builders SHARED
    "src/builder/impl/tree_resource_builder.cpp"
)
target_link_libraries(tree_builders auto_apms_behavior_tree)

# In the original package, we need to manually include this before being able to call the register_nodes macro.
# However, it is automatically available for all downstream packages.
include("cmake/register_builders.cmake")

auto_apms_behavior_tree_register_builders(tree_builders
    "auto_apms_behavior_tree::TreeResourceBuilder"
)

#
# ------------ Utility Executables --------------
#

# Executable to run a default tree executor server node. Also available as a typical ROS node component.
rclcpp_components_register_node(auto_apms_behavior_tree
    PLUGIN "auto_apms_behavior_tree::TreeExecutorServer"
    EXECUTABLE "tree_executor"
)

# Create a file containing a complete behavior tree node plugin load manifest
add_executable(create_node_manifest "src/cli/internal/create_node_manifest.cpp")
target_link_libraries(create_node_manifest auto_apms_behavior_tree)
target_compile_definitions(create_node_manifest PRIVATE
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE}"
)

# Generate the node model according to a behavior tree node plugin manifest
add_executable(generate_node_model "src/cli/internal/generate_node_model.cpp")
target_link_libraries(generate_node_model auto_apms_behavior_tree)

# Run a behavior tree registered with the ament_index
add_executable(run_behavior_tree "src/cli/run_behavior_tree.cpp")
target_link_libraries(run_behavior_tree auto_apms_behavior_tree)

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
    rclcpp_components
    pluginlib
    behaviortree_cpp
    yaml_cpp_vendor
    TinyXML2
    auto_apms_core
    auto_apms_interfaces
    generate_parameter_library
)

###### Non export libraries ######
install(
    TARGETS
    tree_nodes
    tree_builders
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

###### Publicly available cli tools ######
install(
    TARGETS
    run_behavior_tree
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
