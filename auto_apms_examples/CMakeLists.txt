cmake_minimum_required(VERSION 3.9)
project(auto_apms_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_mission REQUIRED)

#
# ------------ Example Executables --------------
#

macro(add_and_install_example_executable source_file)
    get_filename_component(example_target_name "${source_file}" NAME_WE)
    add_executable(${example_target_name} ${source_file})
    ament_target_dependencies(${example_target_name} ${ARGN})
    target_compile_definitions(${example_target_name} PRIVATE EXAMPLE_NAME="${example_target_name}")
    install(TARGETS ${example_target_name} DESTINATION "lib/${PROJECT_NAME}")
endmacro()

add_and_install_example_executable("src/task_ex1.cpp" auto_apms_util auto_apms_interfaces)

#
# ------------ Example Behavior Trees --------------
#

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/relative_goto/relative_goto.xml"
    NODE_MANIFEST
    "config/behavior_tree/relative_goto/node_plugins.yaml"
)

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/px4_contingency_mission/bringup.xml"
    "config/behavior_tree/px4_contingency_mission/mission.xml"
    "config/behavior_tree/px4_contingency_mission/shutdown.xml"
    "config/behavior_tree/px4_contingency_mission/contingency/contingency_land.xml"
    "config/behavior_tree/px4_contingency_mission/contingency/contingency_hold.xml"
    "config/behavior_tree/px4_contingency_mission/emergency/emergency_rtl.xml"
    NODE_MANIFEST
    "auto_apms_behavior_tree::builtin_nodes"
    "auto_apms_px4::px4_behavior_tree_nodes"
)

#
# ------------ Example Missions --------------
#

auto_apms_mission_register_missions(
    "config/mission/px4_contingency_mission.yaml"
)


ament_package()
