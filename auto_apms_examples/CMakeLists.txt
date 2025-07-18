cmake_minimum_required(VERSION 3.9)
project(auto_apms_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(auto_apms_util REQUIRED)
find_package(auto_apms_interfaces REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)
find_package(auto_apms_mission REQUIRED)

# Optional packages
find_package(auto_apms_px4 QUIET)
if(NOT auto_apms_px4_FOUND)
    message(WARNING "auto_apms_px4 not found. No PX4 examples will be generated.")
endif()

#
# ------------ Example Libraries --------------
#

add_library(example_nodes SHARED
    "src/simple_skill_node.cpp"
)
ament_target_dependencies(example_nodes
    rclcpp_components
    auto_apms_interfaces  # Example ROS 2 interfaces. Only added for clarity because auto_apms_behavior_tree already depends on this
    auto_apms_util  # ActionWrapper. Only added for clarity because auto_apms_behavior_tree already depends on this
    auto_apms_behavior_tree
)

add_library(example_build_handlers SHARED
    "src/simple_skill_build_handler.cpp"
)
ament_target_dependencies(example_build_handlers
    auto_apms_behavior_tree
)

auto_apms_behavior_tree_declare_nodes(example_nodes
    "auto_apms_examples::SimpleSkillClient"
    NODE_MANIFEST
    "config/behavior_tree/simple_skill_node_manifest.yaml"
    NODE_MODEL_HEADER_TARGET example_build_handlers
)

auto_apms_behavior_tree_declare_build_handlers(example_build_handlers
    "auto_apms_examples::SimpleSkillBuildHandler"
)

#
# ------------ Example Executables --------------
#

rclcpp_components_register_node(example_nodes
    PLUGIN "auto_apms_examples::SimpleSkillServer"
    EXECUTABLE "simple_skill_server"
)

macro(add_and_install_example_executable source_file)
    get_filename_component(example_target_name "${source_file}" NAME_WE)
    add_executable(${example_target_name} ${source_file})
    ament_target_dependencies(${example_target_name} ${ARGN})
    target_compile_definitions(${example_target_name} PRIVATE EXAMPLE_NAME="${example_target_name}")
    install(TARGETS ${example_target_name} DESTINATION "lib/${PROJECT_NAME}")
endmacro()

#
# ------------ Example Resources --------------
#

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/simple_skill_tree.xml"
    NODE_MANIFEST
    "${PROJECT_NAME}::example_nodes"
)

auto_apms_behavior_tree_declare_trees(
    "config/behavior_tree/demo.xml"
    "config/behavior_tree/pyrobosim/hogwarts.xml"
    "config/behavior_tree/pyrobosim/hogwarts_contingency.xml"
    NODE_MANIFEST
    # We combine previously installed node manifests
    "auto_apms_behavior_tree::behavior_tree_nodes"
    "auto_apms_simulation::behavior_tree_nodes"
)
auto_apms_mission_register_missions(
    "config/mission/pyrobosim_hogwarts_mission.yaml"
)

if(auto_apms_px4_FOUND)
    add_and_install_example_executable("src/px4_takeoff_land.cpp" auto_apms_util auto_apms_interfaces)

    auto_apms_behavior_tree_declare_trees(
        "config/behavior_tree/px4/relative_goto/px4_relative_goto.xml"
        NODE_MANIFEST
        "auto_apms_behavior_tree::behavior_tree_nodes"
        "auto_apms_px4::behavior_tree_nodes"
    )
    auto_apms_behavior_tree_declare_trees(
        "config/behavior_tree/px4/contingency_mission/bringup.xml"
        "config/behavior_tree/px4/contingency_mission/mission.xml"
        "config/behavior_tree/px4/contingency_mission/shutdown.xml"
        "config/behavior_tree/px4/contingency_mission/contingency/contingency_land.xml"
        "config/behavior_tree/px4/contingency_mission/contingency/contingency_hold.xml"
        "config/behavior_tree/px4/contingency_mission/emergency/emergency_rtl.xml"
        NODE_MANIFEST
        "auto_apms_behavior_tree::behavior_tree_nodes"
        "auto_apms_px4::behavior_tree_nodes"
    )
    auto_apms_mission_register_missions(
        "config/mission/px4_contingency_mission.yaml"
    )
endif()

#
# ------------ Packaging --------------
#

install(
    DIRECTORY "launch"
    DESTINATION "share/${PROJECT_NAME}"
)

install(
    TARGETS
    example_nodes
    example_build_handlers
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)


ament_package()
