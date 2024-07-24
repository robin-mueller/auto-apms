cmake_minimum_required(VERSION 3.8)
project(uas_behavior_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ACTION_SOURCES
    action/BTExecutorCommand.action
    action/LaunchBTExecutor.action)
set(SERVICE_SOURCES
    srv/UploadBehaviorTree.srv)

rosidl_generate_interfaces(${PROJECT_NAME} ${ACTION_SOURCES} ${SERVICE_SOURCES})

ament_package()
