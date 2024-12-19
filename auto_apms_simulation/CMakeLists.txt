cmake_minimum_required(VERSION 3.11)
project(auto_apms_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(auto_apms_behavior_tree_core REQUIRED)

set(_AUTO_APMS_SIMULATION__NODE_NAME "pyrobosim")
set(_AUTO_APMS_SIMULATION__SERVICE_NAME__SET_LOCATION_STATE "set_location_state")
set(_AUTO_APMS_SIMULATION__ACTION_NAME__EXECUTE_TASK_ACTION "execute_action")

#
# ------------ PyRoboSim dependency --------------
#

include(FetchContent)
set(PYROBOSIM_VERSION "3.2.1")
FetchContent_Declare(
    pyrobosim
    GIT_REPOSITORY "https://github.com/sea-bass/pyrobosim.git"
    GIT_TAG        "main"
    GIT_SHALLOW    ON
    GIT_PROGRESS   ON
)
set(PYROBOSIM_SOURCE_DIR "${FETCHCONTENT_BASE_DIR}/pyrobosim-src")
FetchContent_Declare(
    pyrobosim_msgs
    SOURCE_DIR "${PYROBOSIM_SOURCE_DIR}/pyrobosim_msgs"
)
FetchContent_MakeAvailable(pyrobosim pyrobosim_msgs)
ament_python_install_package(
    pyrobosim
    PACKAGE_DIR "${PYROBOSIM_SOURCE_DIR}/pyrobosim/pyrobosim"
    VERSION "${PYROBOSIM_VERSION}"
)
ament_python_install_package(
    pyrobosim_ros
    PACKAGE_DIR "${PYROBOSIM_SOURCE_DIR}/pyrobosim_ros/pyrobosim_ros"
    VERSION "${PYROBOSIM_VERSION}"
)
rosidl_get_typesupport_target(pyrobosim_msgs_cpp_typesupport_target
  "pyrobosim_msgs" rosidl_typesupport_cpp)

#
# ------------ Library --------------
#

add_library(auto_apms_simulation SHARED
    "src/util.cpp"
)
target_include_directories(auto_apms_simulation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(auto_apms_simulation
    "${pyrobosim_msgs_cpp_typesupport_target}"
)

#
# ------------ Behavior tree nodes --------------
#

add_library(sim_behavior_tree_nodes SHARED
    "src/node/set_location_state.cpp"
    "src/node/execute_task_action.cpp"
)
target_link_libraries(sim_behavior_tree_nodes
    "${pyrobosim_msgs_cpp_typesupport_target}"
    auto_apms_simulation
)
ament_target_dependencies(sim_behavior_tree_nodes
    auto_apms_behavior_tree_core
)
auto_apms_behavior_tree_declare_nodes(sim_behavior_tree_nodes
    "auto_apms_simulation::SetLocationState"
    "auto_apms_simulation::NavigateToLocation"
    NODE_MANIFEST
    "config/sim_behavior_tree_nodes.yaml.in"
)

#
# ------------ Packaging --------------
#

###### Export libraries ######
install(
    TARGETS
    auto_apms_simulation
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
)

ament_python_install_package(
    ${PROJECT_NAME}
)

install(
    TARGETS
    sim_behavior_tree_nodes
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(PROGRAMS
  "scripts/run_simulation.py"
  DESTINATION "lib/${PROJECT_NAME}"
)

ament_package()
