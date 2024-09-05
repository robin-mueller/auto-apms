cmake_minimum_required(VERSION 3.8)
project(px4_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(px4_behavior_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

#
# ------------ PX4 Behavior Library | Core --------------
#

option(PX4_BEHAVIOR_DEBUG_LOGGING "Enable debug logging severity" ON)

set(_PX4_BEHAVIOR_EXECUTABLES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_DIR_NAME "bt_node_plugins")
set(_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME "px4_behavior__bt_node_plugins")
set(_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_DIR_NAME "behavior_tree")
set(_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_TYPE_NAME "px4_behavior__behavior_trees")

add_library(px4_behavior SHARED
    src/task/vehicle_command_client.cpp
    src/task/arm_disarm.cpp
    src/task/enable_hold.cpp
    src/task/goto.cpp
    src/task/land.cpp
    src/task/takeoff.cpp
    src/task/rtl.cpp
    src/task/mission.cpp
    src/bt_factory.cpp
    src/get_resource.cpp
    src/bt_executor.cpp
    src/bt_executor_client.cpp
    src/ros2_bt_observer.cpp)
target_include_directories(px4_behavior PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_behavior
    rclcpp 
    rclcpp_action 
    rclcpp_components 
    behaviortree_cpp
    behaviortree_ros2
    px4_behavior_interfaces
    px4_msgs 
    px4_ros2_cpp 
    Eigen3 
    tf2_eigen 
    yaml-cpp)
target_link_libraries(px4_behavior ${YAML_CPP_LIBRARIES})
target_compile_definitions(px4_behavior PRIVATE
    _PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_DIR_NAME="${_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_DIR_NAME}"
    _PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME="${_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME}"
    _PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_DIR_NAME="${_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_DIR_NAME}"
    _PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_TYPE_NAME="${_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_TYPE_NAME}")
if(PX4_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(px4_behavior PRIVATE DEBUG_LOGGING)
endif()

rclcpp_components_register_nodes(px4_behavior 
    "px4_behavior::ArmDisarmTask"
    "px4_behavior::EnableHoldTask"
    "px4_behavior::GoToTask"
    "px4_behavior::LandTask"
    "px4_behavior::TakeoffTask"
    "px4_behavior::RTLTask"
    "px4_behavior::MissionTask")

#
# ------------ PX4 Behavior Library | Executables --------------
#

# Command line tool to create a file containing the behavior tree node models required for Groot
add_executable(generate_bt_node_model src/cli/generate_bt_node_model.cpp)
target_link_libraries(generate_bt_node_model px4_behavior)
if(PX4_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(generate_bt_node_model PRIVATE DEBUG_LOGGING)
endif()

# Command line tool to register a behavior tree with an executor
add_executable(upload_tree src/cli/upload_tree.cpp)
target_link_libraries(upload_tree px4_behavior)
if(PX4_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(upload_tree PRIVATE DEBUG_LOGGING)
endif()

# Command line tool to run the executor of a behavior tree
add_executable(run_executor src/cli/run_executor.cpp)
target_link_libraries(run_executor px4_behavior)
if(PX4_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(run_executor PRIVATE DEBUG_LOGGING)
endif()

#
# ------------ PX4 Behavior Library | Standard Behavior Tree Functionality --------------
#

# Preconfigured behavior tree plugins library
add_library(behavior_tree_nodes SHARED
    src/behavior_tree_nodes/action/arm.cpp
    src/behavior_tree_nodes/action/disarm.cpp
    src/behavior_tree_nodes/action/enable_hold.cpp
    src/behavior_tree_nodes/action/goto_vec.cpp
    src/behavior_tree_nodes/action/goto.cpp
    src/behavior_tree_nodes/action/halt_executor.cpp
    src/behavior_tree_nodes/action/land.cpp
    src/behavior_tree_nodes/action/launch_executor.cpp
    src/behavior_tree_nodes/action/load_bt.cpp
    src/behavior_tree_nodes/action/mission.cpp
    src/behavior_tree_nodes/action/pause_executor.cpp
    src/behavior_tree_nodes/action/resume_executor.cpp
    src/behavior_tree_nodes/action/rtl.cpp
    src/behavior_tree_nodes/action/takeoff.cpp
    src/behavior_tree_nodes/action/throw_exception.cpp
    src/behavior_tree_nodes/srv/upload_bt.cpp
    src/behavior_tree_nodes/sub/glob_pos.cpp)
target_link_libraries(behavior_tree_nodes px4_behavior)
ament_target_dependencies(behavior_tree_nodes
    px4_behavior_interfaces
    px4_msgs
    Eigen3)

include(cmake/register_plugins.cmake)

px4_behavior_register_plugins(behavior_tree_nodes
    "px4_behavior::ArmAction"
    "px4_behavior::DisarmAction"
    "px4_behavior::EnableHoldAction"
    "px4_behavior::GoToVectorAction"
    "px4_behavior::GoToAction"
    "px4_behavior::HaltExecutorAction"
    "px4_behavior::LandAction"
    "px4_behavior::LaunchExecutorAction"
    "px4_behavior::LoadBehaviorTreeAction"
    "px4_behavior::MissionAction"
    "px4_behavior::PauseExecutorAction"
    "px4_behavior::ResumeExecutorAction"
    "px4_behavior::RTLAction"
    "px4_behavior::TakeoffAction"
    "px4_behavior::ThrowException"
    "px4_behavior::UploadBehaviorTreeAction"
    "px4_behavior::ReadGlobalPosition")

# For downstream packages this is registered as an extension to ament_package, but here we need to call this script manually
include(cmake/register_resources_hook.cmake)

#
# ------------ PX4 Behavior Library | Packaging --------------
#

install(
    TARGETS px4_behavior
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    TARGETS behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    TARGETS generate_bt_node_model upload_tree run_executor
    DESTINATION ${_PX4_BEHAVIOR_EXECUTABLES_INSTALL_DIR})

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

install(
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})

install(
    DIRECTORY cmake
    DESTINATION share/${PROJECT_NAME})

# Export library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Dependencies
ament_export_dependencies(
    rclcpp 
    rclcpp_action
    behaviortree_cpp
    behaviortree_ros2
    px4_behavior_interfaces
    px4_msgs 
    px4_ros2_cpp
    Eigen3
    tf2_eigen)

ament_package(CONFIG_EXTRAS px4_behavior-extras.cmake.in)
