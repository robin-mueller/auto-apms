cmake_minimum_required(VERSION 3.9)
project(auto_apms_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_mission REQUIRED)

######### Contingency Management

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

# add_library(contingency_management SHARED
    # contingency_management/src/flight_orchestrator.cpp
    # contingency_management/src/safety_monitor.cpp
    # contingency_management/src/contingency_manager.cpp
    # contingency_management/src/mission_manager.cpp
    # contingency_management/src/flight_recorder.cpp
# )
# target_include_directories(contingency_management PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/contingency_management/include>
#     $<INSTALL_INTERFACE:contingency_management/include>)
# ament_target_dependencies(contingency_management
#     auto_apms_behavior_tree
#     rclcpp_components)
# target_link_libraries(contingency_management "${cpp_typesupport_target}")
# install(
#     TARGETS contingency_management
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     RUNTIME DESTINATION bin)

# rclcpp_components_register_nodes(contingency_management
#     "auto_apms::ops_engine::FlightOrchestratorExecutor"
#     "auto_apms::ops_engine::SafetyMonitorExecutor"
#     "auto_apms::ops_engine::ContingencyManagerExecutor"
#     "auto_apms::ops_engine::MissionManagerExecutor")

add_library(behavior_tree_nodes SHARED
    contingency_management/src/behavior_tree_nodes/async_precondition.cpp
    contingency_management/src/behavior_tree_nodes/create_alternate_landing_mission.cpp
    contingency_management/src/behavior_tree_nodes/landing_approach.cpp
    contingency_management/src/behavior_tree_nodes/landing_site_status.cpp
    contingency_management/src/behavior_tree_nodes/system_state.cpp)
target_link_libraries(behavior_tree_nodes "${cpp_typesupport_target}")
ament_target_dependencies(behavior_tree_nodes auto_apms_behavior_tree_core px4_ros2_cpp)
install(
    TARGETS behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

# Behavior tree resources
auto_apms_behavior_tree_declare_nodes(behavior_tree_nodes
    "auto_apms::ops_engine::AsyncPrecondition"
    "auto_apms::ops_engine::CreateAlternateLandingMission"
    "auto_apms::ops_engine::IsApproachingLanding"
    "auto_apms::ops_engine::IsLandingSiteClear"
    "auto_apms::ops_engine::GetSystemState"
)

# auto_apms_behavior_tree_declare_trees("contingency_management/trees/contingency_manager.xml"
#     NODE_MANIFEST
#     "contingency_management/config/contingency_manager_plugins.yaml"
# )
# auto_apms_behavior_tree_declare_trees("contingency_management/trees/flight_orchestrator.xml"
#     NODE_MANIFEST
#     "contingency_management/config/orchestrator_plugins.yaml"
# )
auto_apms_behavior_tree_declare_trees("contingency_management/trees/nominal_mission.xml"
    NODE_MANIFEST
    "contingency_management/config/mission_manager_plugins.yaml"
)
auto_apms_behavior_tree_declare_trees("contingency_management/trees/safety_monitor.xml"
    NODE_MANIFEST
    "contingency_management/config/safety_monitor_plugins.yaml"
)
auto_apms_behavior_tree_declare_trees("contingency_management/trees/alternate_landing_mission.xml")

install(
    DIRECTORY contingency_management/launch
    DESTINATION share/${PROJECT_NAME}
)

#
# ------------ Example Executables --------------
#

macro(add_and_install_example_executable source_file)
    get_filename_component(example_target_name "${source_file}" NAME_WE)
    add_executable(${example_target_name} ${source_file})
    ament_target_dependencies(${example_target_name} ${ARGN})
    target_compile_definitions(${example_target_name} PRIVATE EXAMPLE_NAME="${example_target_name}")
    install(TARGETS ${example_target_name} DESTINATION "lib/${PROJECT_NAME}")
endmacro()

add_and_install_example_executable("src/task_ex1.cpp" auto_apms_util auto_apms_interfaces)

#
# ------------ Example Behavior Trees --------------
#

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/relative_goto/relative_goto.xml"
    NODE_MANIFEST
    "config/behavior_tree/relative_goto/node_plugins.yaml"
)

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/px4_contingency_mission/bringup.xml"
    "config/behavior_tree/px4_contingency_mission/mission.xml"
    "config/behavior_tree/px4_contingency_mission/shutdown.xml"
    "config/behavior_tree/px4_contingency_mission/contingency/contingency_hold.xml"
    NODE_MANIFEST
    "auto_apms_px4::px4_behavior_tree_nodes"
)

#
# ------------ Example Missions --------------
#

auto_apms_mission_register_missions(
    "config/mission/px4_contingency_mission.yaml"
)


ament_package()
