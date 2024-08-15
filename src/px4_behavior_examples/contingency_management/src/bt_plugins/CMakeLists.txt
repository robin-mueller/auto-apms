px4_behavior_register_node_plugin(async_precondition
    async_precondition.cpp)

px4_behavior_register_node_plugin(create_alt_land_mission_action
    create_alternate_landing_mission.cpp)
ament_target_dependencies(create_alt_land_mission_action
    Eigen3
    px4_ros2_cpp)

px4_behavior_register_node_plugin(system_state_sub
    system_state.cpp)
target_link_libraries(system_state_sub "${cpp_typesupport_target}")

px4_behavior_register_node_plugin(landing_approach_sub
    landing_approach.cpp)
target_link_libraries(landing_approach_sub "${cpp_typesupport_target}")

px4_behavior_register_node_plugin(landing_site_status_sub
    landing_site_status.cpp)
target_link_libraries(landing_site_status_sub "${cpp_typesupport_target}")
