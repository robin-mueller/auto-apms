cmake_minimum_required(VERSION 3.8)
project(auto_apms_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

set(_AUTO_APMS_MISSION__DEFAULT_BEHAVIOR_CATEGORY__MISSION "mission")

set(_AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME "orchestrator")
set(_AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME "mission")
set(_AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME "event_monitor")
set(_AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME "event_handler")

#
# ------------ Library --------------
#

add_library(auto_apms_mission SHARED
    "src/mission_config.cpp"
    "src/mission_builder_base.cpp"
)
target_include_directories(auto_apms_mission PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(auto_apms_mission PRIVATE
    _AUTO_APMS_MISSION__DEFAULT_BEHAVIOR_CATEGORY__MISSION="${_AUTO_APMS_MISSION__DEFAULT_BEHAVIOR_CATEGORY__MISSION}"
    _AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME="${_AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME="${_AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME="${_AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME="${_AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME}"
)
ament_target_dependencies(auto_apms_mission
    auto_apms_util
    auto_apms_behavior_tree
)

#
# ------------ Generic mission orchestrator --------------
#

add_library(mission_executors SHARED
    "src/executor_components.cpp"
)
target_compile_definitions(mission_executors PRIVATE
    _AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME="${_AUTO_APMS_MISSION__ORCHESTRATOR_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME="${_AUTO_APMS_MISSION__MISSION_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME="${_AUTO_APMS_MISSION__EVENT_MONITOR_EXECUTOR_NAME}"
    _AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME="${_AUTO_APMS_MISSION__EVENT_HANDLER_EXECUTOR_NAME}"
)
ament_target_dependencies(mission_executors
    rclcpp_components
    auto_apms_behavior_tree
)
rclcpp_components_register_nodes(mission_executors
    "auto_apms_mission::MissionExecutor"
    "auto_apms_mission::EventMonitorExecutor"
    "auto_apms_mission::EventHandlerExecutor"
)
rclcpp_components_register_node(mission_executors
    PLUGIN "auto_apms_mission::OrchestratorExecutor"
    EXECUTABLE "orchestrator"
)

# IMPORTANT: Executor and builder plugins MUST NOT be included by the same shared library, or the executors
# won't be able to load the builders using auto_apms_behavior_tree::TreeBuildHandlerLoader (factory methods
# cannot be discovered by class_loader::MultiLibraryClassLoader)!
add_library(mission_builders SHARED
    "src/build_handler/mission_from_resource.cpp"
    "src/build_handler/mission_from_string.cpp"
    "src/build_handler/multi_node_mission.cpp"
)
target_link_libraries(mission_builders
    auto_apms_mission
)
auto_apms_behavior_tree_declare_build_handlers(mission_builders
    "auto_apms_mission::MissionFromResourceBuildHandler"
    "auto_apms_mission::MissionFromStringBuildHandler"
    "auto_apms_mission::MultiNodeMissionBuildHandler"
)

auto_apms_behavior_tree_declare_trees(
    "config/orchestrator_base.xml"
    NODE_MANIFEST
    "auto_apms_behavior_tree::behavior_tree_nodes"
    MARK_AS_INTERNAL
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
    auto_apms_behavior_tree
)

###### Plugins ######
install(
    TARGETS
    mission_executors
    mission_builders
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY "include/${PROJECT_NAME}"
    DESTINATION "include"
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
