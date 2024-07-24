cmake_minimum_required(VERSION 3.8)
project(uas_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(uas_behavior_interfaces REQUIRED)
find_package(commander_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ------------ UAS Behavior Library --------------

option(UAS_BEHAVIOR_DEBUG_LOGGING "Enable debug logging severity" OFF)

set(UAS_BEHAVIOR_SHARE_DIR_NAME ${PROJECT_NAME})
set(UAS_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG "config")
set(UAS_BEHAVIOR_SHARE_SUBDIR_NAME_TREES "trees")

add_library(uas_behavior SHARED
    src/factory.cpp
    src/get_resource.cpp
    src/bt_executor.cpp
    src/bt_executor_client.cpp
    src/ros2_bt_observer.cpp)
target_include_directories(uas_behavior PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(uas_behavior
    behaviortree_cpp
    behaviortree_ros2
    uas_behavior_interfaces
    yaml-cpp)
target_link_libraries(uas_behavior ${YAML_CPP_LIBRARIES})
target_compile_definitions(uas_behavior PRIVATE
    UAS_BEHAVIOR_SHARE_DIR_NAME="${UAS_BEHAVIOR_SHARE_DIR_NAME}"
    UAS_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG="${UAS_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG}"
    UAS_BEHAVIOR_SHARE_SUBDIR_NAME_TREES="${UAS_BEHAVIOR_SHARE_SUBDIR_NAME_TREES}")
if(UAS_BEHAVIOR_DEBUG_LOGGING)
    target_compile_definitions(uas_behavior PRIVATE DEBUG_LOGGING)
endif()
install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)
install(
    TARGETS uas_behavior
    EXPORT export_uas_behavior
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Make library available to other packages
ament_export_targets(export_uas_behavior HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp behaviortree_ros2 uas_behavior_interfaces)

configure_file(cmake/add_bt_plugin.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/add_bt_plugin.cmake @ONLY)
configure_file(cmake/generate_bt_node_model.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate_bt_node_model.cmake @ONLY)
configure_file(cmake/resource_registration.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/resource_registration.cmake @ONLY)
set(cmake_files
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/add_bt_plugin.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/generate_bt_node_model.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/resource_registration.cmake)

# Include those files in this CMakeLists for later use
foreach(cmake_file ${cmake_files})
    include(${cmake_file})
endforeach()

# ------------ Executables --------------

# Command line tool to create a file that specifies all model definitions of the plugins used
add_executable(create_bt_node_model src/create_bt_node_model.cpp)
target_link_libraries(create_bt_node_model uas_behavior)
install(
    TARGETS create_bt_node_model
    DESTINATION lib/${PROJECT_NAME})

# Command line tool to register a behavior tree with an executor
add_executable(upload_tree src/upload_tree.cpp)
target_link_libraries(upload_tree uas_behavior)
install(
    TARGETS upload_tree
    DESTINATION lib/${PROJECT_NAME})

# Command line tool to run the executor of a behavior tree
add_executable(run_executor src/run_executor.cpp)
target_link_libraries(run_executor uas_behavior)
install(
    TARGETS run_executor
    DESTINATION lib/${PROJECT_NAME})

# ------------ Standard Behavior Tree Functionality --------------

# Preconfigured behavior tree plugins
add_subdirectory(src/bt_plugins)

# ------------- Examples -----------------

# Emulate cmake configuration as if uas_behavior was imported
add_library(uas_behavior::uas_behavior ALIAS uas_behavior) # ROS2 creates namespaced targets for exported libraries
set(UAS_BEHAVIOR_CREATE_BT_NODE_MODEL_EXE_PATH create_bt_node_model) # Use the target of this CMakeLists.txt to create the bt node model since the install is not available yet

# Register behavior tree resources
uas_behavior_register_config_directory(config)
uas_behavior_register_trees_directory(trees)

# Make sure to use the binary path plugin library when generating the model definition in the same package, since the install dir is not available yet
uas_behavior_generate_bt_node_model(
    "${CMAKE_CURRENT_SOURCE_DIR}/config/example_goto_bt_node_config.yaml" "${CMAKE_CURRENT_BINARY_DIR}/src/bt_plugins")

set(EXAMPLE_TARGET_NAME example_goto)
add_executable(${EXAMPLE_TARGET_NAME} examples/goto.cpp)
target_link_libraries(${EXAMPLE_TARGET_NAME} uas_behavior)
target_compile_definitions(${EXAMPLE_TARGET_NAME} PRIVATE EXAMPLE_NAME="${EXAMPLE_TARGET_NAME}")
install(TARGETS ${EXAMPLE_TARGET_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS ${cmake_files})
