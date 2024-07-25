cmake_minimum_required(VERSION 3.8)
project(px4_behavior_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ACTION_SOURCES
    action/ArmDisarm.action
    action/EnableHold.action
    action/GoTo.action
    action/Land.action
    action/RTL.action
    action/Takeoff.action
    action/Mission.action
    action/LaunchBTExecutor.action
    action/BTExecutorCommand.action)
set(MESSAGE_SOURCES
    msg/ForceContingency.msg
    msg/LandingApproach.msg
    msg/LandingSiteStatus.msg
    msg/ContingencyEvent.msg
    msg/SystemState.msg)
set(SERVICE_SOURCES
    srv/UploadBehaviorTree.srv)

rosidl_generate_interfaces(${PROJECT_NAME} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES}
    DEPENDENCIES geometry_msgs)

ament_package()
