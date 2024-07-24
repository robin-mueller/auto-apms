cmake_minimum_required(VERSION 3.8)
project(contingency_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(uas_behavior REQUIRED)
find_package(contingency_manager_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(contingency_manager SHARED
    src/flight_orchestrator.cpp
    src/safety_monitor.cpp
    src/contingency_manager.cpp
    src/mission_manager.cpp
    src/flight_recorder.cpp)
target_include_directories(contingency_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(contingency_manager
    rclcpp_action
    rclcpp_components
    uas_behavior
    contingency_manager_interfaces
    px4_msgs
    px4_ros2_cpp)
install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)
install(
    TARGETS contingency_manager
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
rclcpp_components_register_nodes(contingency_manager "contingency_manager::FlightOrchestratorExecutor")
rclcpp_components_register_nodes(contingency_manager "contingency_manager::SafetyMonitorExecutor")
rclcpp_components_register_nodes(contingency_manager "contingency_manager::ContingencyManagerExecutor")
rclcpp_components_register_nodes(contingency_manager "contingency_manager::MissionManagerExecutor")

rclcpp_components_register_node(contingency_manager PLUGIN "contingency_manager::FlightRecorderNode" EXECUTABLE flight_recorder)

# rclcpp_components_register_node(contingency_manager PLUGIN "contingency_manager::MissionManagerExecutor" EXECUTABLE node_debug_exe)

# Launch files
install(
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})

# Register behaviortree resources
add_subdirectory(src/bt_plugins)
uas_behavior_register_config_directory(config)
uas_behavior_register_trees_directory(trees)

# Make sure to provide the binary path of the shared library object files as an argument
# when generating the model definition in the same package.
# The install dir is not yet available (it is generated after the build step).
set(BT_PLUGINS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/src/bt_plugins)
uas_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/mission_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
uas_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/contingency_manager_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
uas_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/orchestrator_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
uas_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/safety_monitor_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")

ament_package()
