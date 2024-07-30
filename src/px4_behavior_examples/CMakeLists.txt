cmake_minimum_required(VERSION 3.8)
project(px4_behavior_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_behavior REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#
# ------------ PX4 Behavior Examples | Interfaces --------------
#

# Additional ROS2 Interfaces
set(ACTION_SOURCES
    )
set(MESSAGE_SOURCES
    contingency_management/interfaces/msg/ForceContingency.msg
    contingency_management/interfaces/msg/LandingApproach.msg
    contingency_management/interfaces/msg/LandingSiteStatus.msg
    contingency_management/interfaces/msg/ContingencyEvent.msg
    contingency_management/interfaces/msg/SystemState.msg)
set(SERVICE_SOURCES
    )

rosidl_generate_interfaces(${PROJECT_NAME} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

#
# ------------ PX4 Behavior Examples | Commander --------------
#

set(EXAMPLE_TARGET_NAME contingency)
add_executable(${EXAMPLE_TARGET_NAME} commander/src/contingency.cpp)
ament_target_dependencies(${EXAMPLE_TARGET_NAME}
    px4_behavior)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PRIVATE EXAMPLE_NAME="example_${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

#
# ------------ PX4 Behavior Examples | Goto --------------
#

set(EXAMPLE_TARGET_NAME goto)
add_executable(${EXAMPLE_TARGET_NAME} goto/src/goto.cpp)
ament_target_dependencies(${EXAMPLE_TARGET_NAME}
    px4_behavior)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PRIVATE EXAMPLE_NAME="example_${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

px4_behavior_register_config_directory(goto/config)
px4_behavior_register_trees_directory(goto/trees)

px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/goto/config/example_goto_bt_node_config.yaml")

#
# ------------ PX4 Behavior Examples | Contingency Management --------------
#

add_library(contingency_management SHARED
    contingency_management/src/flight_orchestrator.cpp
    contingency_management/src/safety_monitor.cpp
    contingency_management/src/contingency_manager.cpp
    contingency_management/src/mission_manager.cpp
    contingency_management/src/flight_recorder.cpp)
target_include_directories(contingency_management PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/include>
    $<INSTALL_INTERFACE:contingency_management/include>)
ament_target_dependencies(contingency_management
    px4_behavior
    rclcpp_components)
target_link_libraries(contingency_management "${cpp_typesupport_target}")
install(
    TARGETS contingency_management
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

rclcpp_components_register_nodes(contingency_management "FlightOrchestratorExecutor")
rclcpp_components_register_nodes(contingency_management "SafetyMonitorExecutor")
rclcpp_components_register_nodes(contingency_management "ContingencyManagerExecutor")
rclcpp_components_register_nodes(contingency_management "MissionManagerExecutor")

# Behavior tree resources
px4_behavior_register_config_directory(contingency_management/config)
px4_behavior_register_trees_directory(contingency_management/trees)

# Custom behavior tree plugins (It is important to do this before generating the model)
add_subdirectory(contingency_management/src/bt_plugins)

# Make sure to provide the binary path of the shared library object files as an argument
# when generating the model definition in the same package.
# The install dir is not yet available (it is generated after the build step).
set(EXTRA_PLUGINS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/contingency_management/src/bt_plugins)
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/config/mission_bt_node_config.yaml")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/config/contingency_manager_bt_node_config.yaml" EXTRA_PLUGIN_DIR "${EXTRA_PLUGINS_BINARY_DIR}")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/config/orchestrator_bt_node_config.yaml")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/config/safety_monitor_bt_node_config.yaml" EXTRA_PLUGIN_DIR "${EXTRA_PLUGINS_BINARY_DIR}")



ament_package()
