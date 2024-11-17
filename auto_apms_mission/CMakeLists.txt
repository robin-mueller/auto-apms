cmake_minimum_required(VERSION 3.8)
project(auto_apms_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components)
find_package(auto_apms_behavior_tree REQUIRED)

#
# ------------ Library --------------
#

#
# ------------ Generic mission orchestrator --------------
#

add_library(mission_orchestrator SHARED
    "src/orchestrator.cpp"
)
ament_target_dependencies(mission_orchestrator
    rclcpp_components
    auto_apms_behavior_tree
)
rclcpp_components_register_nodes(mission_orchestrator
    "auto_apms_mission::MissionOrchestrator"
)
auto_apms_behavior_tree_declare_trees(
    "config/orchestrator_base.xml"
    NODE_MANIFEST
    "auto_apms_behavior_tree::builtin_nodes"
)

#
# ------------ Tree build handlers --------------
#

#
# ------------ Packaging --------------
#

install(
    TARGETS
    mission_orchestrator
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY "launch"
    DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
