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

set(PX4_BEHAVIOR_SHARE_DIR_NAME ${PROJECT_NAME})
set(PX4_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG "config")
set(PX4_BEHAVIOR_SHARE_SUBDIR_NAME_TREES "trees")

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
    src/ros2_bt_observer.cpp
    src/flight_recorder.cpp)
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
    PX4_BEHAVIOR_SHARE_DIR_NAME="${PX4_BEHAVIOR_SHARE_DIR_NAME}"
    PX4_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG="${PX4_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG}"
    PX4_BEHAVIOR_SHARE_SUBDIR_NAME_TREES="${PX4_BEHAVIOR_SHARE_SUBDIR_NAME_TREES}")
if(PX4_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(px4_behavior PRIVATE DEBUG_LOGGING)
endif()

rclcpp_components_register_nodes(px4_behavior "px4_behavior::ArmDisarmManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::EnableHoldManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::GoToManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::LandManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::TakeoffManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::RTLManeuver")
rclcpp_components_register_nodes(px4_behavior "px4_behavior::MissionManeuver")

rclcpp_components_register_node(px4_behavior PLUGIN "px4_behavior::FlightRecorderNode" EXECUTABLE flight_recorder)
# rclcpp_components_register_node(px4_behavior PLUGIN "px4_behavior::MissionManagerExecutor" EXECUTABLE debug_node)


install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)
install(
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})
install(
    TARGETS px4_behavior
    EXPORT export_px4_behavior
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

# Make library available to other packages
ament_export_targets(export_px4_behavior HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp behaviortree_ros2 px4_behavior_interfaces)

configure_file(cmake/add_bt_plugin.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/add_bt_plugin.cmake @ONLY)
configure_file(cmake/generate_bt_node_model.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate_bt_node_model.cmake @ONLY)
configure_file(cmake/resource_registration.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/resource_registration.cmake @ONLY)
set(cmake_files
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/add_bt_plugin.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate_bt_node_model.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/resource_registration.cmake)

# Include those files in this CMakeLists for later use
foreach(cmake_file ${cmake_files})
    include(${cmake_file})
endforeach()

#
# ------------ PX4 Behavior Library | Executables --------------
#

# Command line tool to create a file that specifies all model definitions of the plugins used
add_executable(create_bt_node_model src/cli/create_bt_node_model.cpp)
target_link_libraries(create_bt_node_model px4_behavior)
install(
    TARGETS create_bt_node_model
    DESTINATION lib/${PROJECT_NAME})

# Command line tool to register a behavior tree with an executor
add_executable(upload_tree src/cli/upload_tree.cpp)
target_link_libraries(upload_tree px4_behavior)
install(
    TARGETS upload_tree
    DESTINATION lib/${PROJECT_NAME})

# Command line tool to run the executor of a behavior tree
add_executable(run_executor src/cli/run_executor.cpp)
target_link_libraries(run_executor px4_behavior)
install(
    TARGETS run_executor
    DESTINATION lib/${PROJECT_NAME})

#
# ------------ PX4 Behavior Library | Standard Behavior Tree Functionality --------------
#

# Preconfigured behavior tree plugins
add_subdirectory(src/bt_plugins)

#
# ------------- PX4 Behavior Library | Examples -----------------
#

# Add example sources
add_library(px4_behavior_examples SHARED
    examples/contingency_management/src/flight_orchestrator.cpp
    examples/contingency_management/src/safety_monitor.cpp
    examples/contingency_management/src/contingency_manager.cpp
    examples/contingency_management/src/mission_manager.cpp)
target_link_libraries(px4_behavior_examples px4_behavior)
install(
    TARGETS px4_behavior_examples
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

rclcpp_components_register_nodes(px4_behavior_examples "FlightOrchestratorExecutor")
rclcpp_components_register_nodes(px4_behavior_examples "SafetyMonitorExecutor")
rclcpp_components_register_nodes(px4_behavior_examples "ContingencyManagerExecutor")
rclcpp_components_register_nodes(px4_behavior_examples "MissionManagerExecutor")

set(EXAMPLE_TARGET_NAME example_goto)
add_executable(${EXAMPLE_TARGET_NAME} examples/goto/src/goto.cpp)
target_link_libraries(${EXAMPLE_TARGET_NAME} px4_behavior)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PRIVATE EXAMPLE_NAME="${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

set(EXAMPLE_TARGET_NAME example_contingency)
add_executable(${EXAMPLE_TARGET_NAME} examples/commander/contingency.cpp)
target_link_libraries(${EXAMPLE_TARGET_NAME} px4_behavior)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PRIVATE EXAMPLE_NAME="${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

# Emulate cmake configuration as if px4_behavior was imported
add_library(px4_behavior::px4_behavior ALIAS px4_behavior) # ROS2 creates namespaced targets for exported libraries
set(PX4_BEHAVIOR_CREATE_BT_NODE_MODEL_EXE_PATH create_bt_node_model) # Use the target of this CMakeLists.txt to create the bt node model since the install is not available yet

# Register behavior tree resources
px4_behavior_register_config_directory(examples/contingency_management/config)
px4_behavior_register_trees_directory(examples/contingency_management/trees)

# Make sure to provide the binary path of the shared library object files as an argument
# when generating the model definition in the same package.
# The install dir is not yet available (it is generated after the build step).
set(BT_PLUGINS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/src/bt_plugins)
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/contingency_management/config/mission_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/contingency_management/config/contingency_manager_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/contingency_management/config/orchestrator_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/contingency_management/config/safety_monitor_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")
px4_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/goto/config/example_goto_bt_node_config.yaml" "${BT_PLUGINS_BINARY_DIR}")

ament_package(CONFIG_EXTRAS ${cmake_files})
