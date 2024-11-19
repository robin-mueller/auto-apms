cmake_minimum_required(VERSION 3.8)
project(auto_apms_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

set(_AUTO_APMS_MISSION__RESOURCE_TYPE_NAME__MISSION_CONFIG "auto_apms_mission__config")

#
# ------------ Library --------------
#

add_library(auto_apms_mission SHARED
    "src/mission_configuration.cpp"
)
target_include_directories(auto_apms_mission PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(auto_apms_mission PRIVATE
    _AUTO_APMS_MISSION__RESOURCE_TYPE_NAME__MISSION_CONFIG="${_AUTO_APMS_MISSION__RESOURCE_TYPE_NAME__MISSION_CONFIG}"
)
ament_target_dependencies(auto_apms_mission
    auto_apms_util
)

#
# ------------ Generic mission orchestrator --------------
#

add_library(mission_orchestrator SHARED
    "src/mission_orchestrator.cpp"
    "src/mission_build_handler.cpp"
)
target_link_libraries(mission_orchestrator
    auto_apms_mission
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
auto_apms_behavior_tree_declare_build_handlers(mission_orchestrator
    "auto_apms_mission::MissionBuildHandler"
)

#
# ------------ Packaging --------------
#

###### Export libraries ######
install(
    TARGETS
    auto_apms_mission
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    auto_apms_util
)

###### Plugins ######
install(
    TARGETS
    mission_orchestrator
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY "cmake"
    DESTINATION "share/${PROJECT_NAME}"
)

install(
    DIRECTORY "launch"
    DESTINATION "share/${PROJECT_NAME}"
)

ament_package(CONFIG_EXTRAS "auto_apms_mission-extras.cmake.in")
