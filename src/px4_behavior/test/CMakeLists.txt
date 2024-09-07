find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_lint_auto_find_test_dependencies()

find_package(rosidl_default_generators REQUIRED)
add_subdirectory(interfaces)  # CMAKE_CURRENT_SOURCE_DIR must be at root of action/srv/msg directories for rosidl_generate_interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

add_library(test_helper SHARED
    helper/src/util.cpp)
target_include_directories(test_helper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/helper/include>
)
target_link_libraries(test_helper 
    px4_behavior_core)

# Integration tests
# ament_add_gtest(${PROJECT_NAME}_integration_tests
        
# )
# target_include_directories(${PROJECT_NAME}_integration_tests PRIVATE
#      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )
# target_link_libraries(${PROJECT_NAME}_integration_tests px4_behavior_core)

# Unit tests
ament_add_gtest(${PROJECT_NAME}_unit_tests
    unit/src/task.cpp

    unit/src/main.cpp
)
target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_unit_tests 
    px4_behavior_core
    test_helper
    "${cpp_typesupport_target}")