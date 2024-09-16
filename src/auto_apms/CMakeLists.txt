cmake_minimum_required(VERSION 3.8)
project(auto_apms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

#
# ------------ PX4 Behavior Library | Core --------------
#

option(AUTO_APMS_DEBUG_LOGGING "Enable debug logging severity" ON)

set(_AUTO_APMS_EXECUTABLES_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_DIR_NAME "bt_node_plugins")
set(_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME "auto_apms__bt_node_plugins")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME "behavior_tree")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME "auto_apms__behavior_trees")

add_library(auto_apms_core SHARED
    src/resource/node.cpp
    src/resource/tree.cpp
    src/bt_executor.cpp
    src/bt_executor_client.cpp
    src/ros2_bt_observer.cpp
    src/vehicle_command_client.cpp

    src/task/arm_disarm.cpp
    src/task/enable_hold.cpp
    src/task/goto.cpp
    src/task/land.cpp
    src/task/takeoff.cpp
    src/task/rtl.cpp
    src/task/mission.cpp
)
target_include_directories(auto_apms_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(auto_apms_core
    rclcpp
    rclcpp_action
    rclcpp_components
    behaviortree_cpp
    behaviortree_ros2
    auto_apms_interfaces
    px4_msgs
    px4_ros2_cpp
    Eigen3
    tf2_eigen
    yaml-cpp)
target_link_libraries(auto_apms_core ${YAML_CPP_LIBRARIES})
target_compile_definitions(auto_apms_core PRIVATE
    _AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_DIR_NAME="${_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_DIR_NAME}"
    _AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME="${_AUTO_APMS_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME}"
    _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME="${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME}")
if(AUTO_APMS_DEBUG_LOGGING)
    target_compile_definitions(auto_apms_core PRIVATE DEBUG_LOGGING)
endif()

rclcpp_components_register_nodes(auto_apms_core
    "auto_apms::ArmDisarmTask"
    "auto_apms::EnableHoldTask"
    "auto_apms::GoToTask"
    "auto_apms::LandTask"
    "auto_apms::TakeoffTask"
    "auto_apms::RTLTask"
    "auto_apms::MissionTask")

#
# ------------ PX4 Behavior Library | Executables --------------
#

# Command line tool to create a file containing the behavior tree node models required for Groot
add_executable(generate_bt_node_model src/cli/generate_bt_node_model.cpp)
target_link_libraries(generate_bt_node_model auto_apms_core)
if(AUTO_APMS_DEBUG_LOGGING)
    target_compile_definitions(generate_bt_node_model PRIVATE DEBUG_LOGGING)
endif()

# Command line tool to register a behavior tree with an executor
add_executable(upload_tree src/cli/upload_tree.cpp)
target_link_libraries(upload_tree auto_apms_core)
if(AUTO_APMS_DEBUG_LOGGING)
    target_compile_definitions(upload_tree PRIVATE DEBUG_LOGGING)
endif()

# Command line tool to run the executor of a behavior tree
add_executable(run_executor src/cli/run_executor.cpp)
target_link_libraries(run_executor auto_apms_core)
if(AUTO_APMS_DEBUG_LOGGING)
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
target_link_libraries(behavior_tree_nodes auto_apms_core)
ament_target_dependencies(behavior_tree_nodes
    auto_apms_interfaces
    px4_msgs
    Eigen3)

include(cmake/register_plugins.cmake)

auto_apms_register_plugins(behavior_tree_nodes
    "auto_apms::ArmAction"
    "auto_apms::DisarmAction"
    "auto_apms::EnableHoldAction"
    "auto_apms::GoToVectorAction"
    "auto_apms::GoToAction"
    "auto_apms::HaltExecutorAction"
    "auto_apms::LandAction"
    "auto_apms::LaunchExecutorAction"
    "auto_apms::LoadBehaviorTreeAction"
    "auto_apms::MissionAction"
    "auto_apms::PauseExecutorAction"
    "auto_apms::ResumeExecutorAction"
    "auto_apms::RTLAction"
    "auto_apms::TakeoffAction"
    "auto_apms::ThrowException"
    "auto_apms::UploadBehaviorTreeAction"
    "auto_apms::ReadGlobalPosition")

# For downstream packages this is registered as an extension to ament_package, but here we need to call this script manually
include(cmake/register_resources_hook.cmake)

#
# ------------ PX4 Behavior Library | Testing --------------
#

if(BUILD_TESTING)
    ### Test interfaces and helper ###
    find_package(rosidl_default_generators REQUIRED)
    add_subdirectory(test/interfaces)  # CMAKE_CURRENT_SOURCE_DIR must be at root of 'action' directory for rosidl_generate_interfaces
    rosidl_get_typesupport_target(test_interfaces ${PROJECT_NAME} rosidl_typesupport_cpp)

    add_library(test_helper SHARED
        test/helper/src/util.cpp)
    target_include_directories(test_helper PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/helper/include>)
    target_link_libraries(test_helper auto_apms_core)

    ### Unit tests ###
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(${PROJECT_NAME}_unit_tests
        test/unit/src/task.cpp

        test/unit/src/main.cpp)
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>)
    target_link_libraries(${PROJECT_NAME}_unit_tests
        auto_apms_core
        test_helper
        "${test_interfaces}")

    ### Integration tests ###
    # ament_add_gtest(${PROJECT_NAME}_integration_tests

    # )
    # target_include_directories(${PROJECT_NAME}_integration_tests PRIVATE
    #      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
    # )
    # target_link_libraries(${PROJECT_NAME}_integration_tests auto_apms_core)
endif()

#
# ------------ PX4 Behavior Library | Packaging --------------
#

install(
    TARGETS auto_apms_core
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
    DESTINATION ${_AUTO_APMS_EXECUTABLES_INSTALL_DIR})

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
    auto_apms_interfaces
    px4_msgs
    px4_ros2_cpp
    Eigen3
    tf2_eigen)

ament_package(CONFIG_EXTRAS auto_apms-extras.cmake.in)
