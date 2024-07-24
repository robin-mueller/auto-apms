cmake_minimum_required(VERSION 3.8)
project(contingency_manager_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(commander_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MESSAGE_SOURCES
    msg/ForceContingency.msg
    msg/LandingApproach.msg
    msg/LandingSiteStatus.msg
    msg/ContingencyEvent.msg
    msg/SystemState.msg)

rosidl_generate_interfaces(${PROJECT_NAME} ${MESSAGE_SOURCES}
    DEPENDENCIES commander_interfaces px4_msgs)

ament_package()
