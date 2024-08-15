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

option(PX4_BEHAVIOR_DEBUG_LOGGING "Enable debug logging severity" OFF)

set(_PX4_BEHAVIOR_EXECUTABLES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME "plugin_config")
set(_PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME "behavior_tree")

add_library(px4_behavior SHARED
    src/commander/vehicle_command_client.cpp
    src/commander/arm_disarm.cpp
    src/commander/enable_hold.cpp
    src/commander/goto.cpp
    src/commander/land.cpp
    src/commander/takeoff.cpp
    src/commander/rtl.cpp
    src/commander/mission.cpp
    src/factory.cpp
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
    _PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME="${_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME}"
    _PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME="${_PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME}")
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

rclcpp_components_register_node(px4_behavior PLUGIN "px4_behavior::FlightRecorderNode" EXECUTABLE flight_recorder)
# rclcpp_components_register_node(px4_behavior PLUGIN "px4_behavior::MissionManagerExecutor" EXECUTABLE debug_node)

#
# ------------ PX4 Behavior Library | Executables --------------
#

# Command line tool to create a file that specifies all model definitions of the plugins used
add_executable(generate_bt_node_model src/cli/generate_bt_node_model.cpp)
target_link_libraries(generate_bt_node_model px4_behavior)

# Command line tool to register a behavior tree with an executor
add_executable(upload_tree src/cli/upload_tree.cpp)
target_link_libraries(upload_tree px4_behavior)

# Command line tool to run the executor of a behavior tree
add_executable(run_executor src/cli/run_executor.cpp)
target_link_libraries(run_executor px4_behavior)

#
# ------------ PX4 Behavior Library | Standard Behavior Tree Functionality --------------
#

# Emulate cmake configuration as if px4_behavior was imported
add_library(px4_behavior::px4_behavior ALIAS px4_behavior) # ROS2 creates namespaced targets for exported libraries
include(cmake/register_node_plugin.cmake)

# Preconfigured behavior tree plugins
add_subdirectory(src/bt_plugins)

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
    tf2_eigen)

ament_package(CONFIG_EXTRAS px4_behavior-extras.cmake.in)
