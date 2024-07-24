cmake_minimum_required(VERSION 3.8)
project(commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(commander_interfaces REQUIRED)

option(COMMANDER_DEBUG_LOGGING "Enable debug logging severity" OFF)

# Library
add_library(commander SHARED
    src/vehicle_command_client.cpp
    src/maneuver/arm_disarm.cpp
    src/maneuver/enable_hold.cpp
    src/maneuver/goto.cpp
    src/maneuver/land.cpp
    src/maneuver/takeoff.cpp
    src/maneuver/rtl.cpp
    src/maneuver/mission.cpp)
target_include_directories(commander PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
if(COMMANDER_DEBUG_LOGGING)
    target_compile_definitions(commander PRIVATE DEBUG_LOGGING)
endif()
ament_target_dependencies(commander
    rclcpp 
    rclcpp_action 
    rclcpp_components 
    px4_msgs 
    px4_ros2_cpp 
    Eigen3 
    tf2_eigen 
    commander_interfaces)

rclcpp_components_register_nodes(commander "commander::ArmDisarmManeuver")
rclcpp_components_register_nodes(commander "commander::EnableHoldManeuver")
rclcpp_components_register_nodes(commander "commander::GoToManeuver")
rclcpp_components_register_nodes(commander "commander::LandManeuver")
rclcpp_components_register_nodes(commander "commander::TakeoffManeuver")
rclcpp_components_register_nodes(commander "commander::RTLManeuver")
rclcpp_components_register_nodes(commander "commander::MissionManeuver")

install(TARGETS commander
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY 
    include/${PROJECT_NAME}
    DESTINATION include)

# Examples
set(EXAMPLE_TARGET_NAME example_contingency)
add_executable(${EXAMPLE_TARGET_NAME}
    examples/contingency.cpp)
target_link_libraries(${EXAMPLE_TARGET_NAME} commander)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PUBLIC 
    EXAMPLE_NAME="${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} 
    DESTINATION lib/${PROJECT_NAME})

# Launch files
install(DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME})

ament_package()
