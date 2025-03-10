cmake_minimum_required(VERSION 3.8)
project(auto_apms_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(TEST_SOURCES
    "action/TestActionWrapper.action"
)

set(EXAMPLE_SOURCES
    "action/ExampleSimpleSkill.action"
)

set(MESSAGE_SOURCES
)
set(SERVICE_SOURCES
)
set(ACTION_SOURCES
    "action/ArmDisarm.action"
    "action/EnableHold.action"
    "action/GoTo.action"
    "action/Land.action"
    "action/RTL.action"
    "action/Takeoff.action"
    "action/Mission.action"
    "action/StartTreeExecutor.action"
    "action/CommandTreeExecutor.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${TEST_SOURCES} ${EXAMPLE_SOURCES} ${ACTION_SOURCES} ${SERVICE_SOURCES} ${MESSAGE_SOURCES}
    DEPENDENCIES geometry_msgs
)

ament_package()
