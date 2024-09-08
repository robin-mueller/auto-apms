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
    "contingency_management/interfaces/msg/ForceContingency.msg"
    "contingency_management/interfaces/msg/LandingApproach.msg"
    "contingency_management/interfaces/msg/LandingSiteStatus.msg"
    "contingency_management/interfaces/msg/ContingencyEvent.msg"
    "contingency_management/interfaces/msg/SystemState.msg")
set(SERVICE_SOURCES
    )

rosidl_generate_interfaces(${PROJECT_NAME} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

#
# ------------ PX4 Behavior Examples | Example sources --------------
#

macro(add_and_install_example_executable source_file)
    get_filename_component(example_target_name "${source_file}" NAME_WE)
    add_executable(${example_target_name} ${source_file})
    ament_target_dependencies(${example_target_name} px4_behavior)
    target_compile_definitions(${example_target_name} PRIVATE EXAMPLE_NAME="${example_target_name}")
    install(TARGETS ${example_target_name} DESTINATION lib/${PROJECT_NAME})
endmacro()

add_and_install_example_executable("src/task_ex1.cpp")
add_and_install_example_executable("src/behavior_ex1.cpp")

# Example behavior tree resources
px4_behavior_register_behavior_tree_file(
    "resource/relative_goto.xml"
    PLUGIN_CONFIGS
    "resource/relative_goto_plugins.yaml"
    TREE_IDS
    "MainTree")

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

rclcpp_components_register_nodes(contingency_management
    "px4_behavior::ops_engine::FlightOrchestratorExecutor"
    "px4_behavior::ops_engine::SafetyMonitorExecutor"
    "px4_behavior::ops_engine::ContingencyManagerExecutor"
    "px4_behavior::ops_engine::MissionManagerExecutor")

rclcpp_components_register_node(contingency_management PLUGIN "px4_behavior::ops_engine::FlightRecorderNode" EXECUTABLE flight_recorder)
# rclcpp_components_register_node(contingency_management PLUGIN "px4_behavior::ops_engine::MissionManagerExecutor" EXECUTABLE debug_node)

add_library(behavior_tree_nodes SHARED
    contingency_management/src/behavior_tree_nodes/async_precondition.cpp
    contingency_management/src/behavior_tree_nodes/create_alternate_landing_mission.cpp
    contingency_management/src/behavior_tree_nodes/landing_approach.cpp
    contingency_management/src/behavior_tree_nodes/landing_site_status.cpp
    contingency_management/src/behavior_tree_nodes/system_state.cpp)
target_link_libraries(behavior_tree_nodes "${cpp_typesupport_target}")
ament_target_dependencies(behavior_tree_nodes px4_behavior)

# Behavior tree resources
px4_behavior_register_plugins(behavior_tree_nodes
    "px4_behavior::ops_engine::AsyncPrecondition"
    "px4_behavior::ops_engine::CreateAlternateLandingMission"
    "px4_behavior::ops_engine::IsApproachingLanding"
    "px4_behavior::ops_engine::IsLandingSiteClear"
    "px4_behavior::ops_engine::GetSystemState")

px4_behavior_register_behavior_tree_file("contingency_management/trees/contingency_manager.xml"
    PLUGIN_CONFIGS
    "contingency_management/config/contingency_manager_plugins.yaml")
px4_behavior_register_behavior_tree_file("contingency_management/trees/flight_orchestrator.xml"
    PLUGIN_CONFIGS
    "contingency_management/config/orchestrator_plugins.yaml")
px4_behavior_register_behavior_tree_file("contingency_management/trees/nominal_mission.xml"
    PLUGIN_CONFIGS
    "contingency_management/config/mission_manager_plugins.yaml")
px4_behavior_register_behavior_tree_file("contingency_management/trees/safety_monitor.xml"
    PLUGIN_CONFIGS
    "contingency_management/config/safety_monitor_plugins.yaml")
px4_behavior_register_behavior_tree_file("contingency_management/trees/alternate_landing_mission.xml"
    TREE_IDS
    "AlternateLandingMission")

install(
    DIRECTORY contingency_management/launch
    DESTINATION share/${PROJECT_NAME})



ament_package()
