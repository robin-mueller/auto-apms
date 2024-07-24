uas_behavior_add_bt_plugin(async_precondition
    control/async_precondition.cpp)

uas_behavior_add_bt_plugin(create_alt_land_mission_action
    action/create_alternate_landing_mission.cpp)
ament_target_dependencies(create_alt_land_mission_action
    Eigen3
    px4_ros2_cpp)

uas_behavior_add_bt_plugin(system_state_sub
    sub/system_state.cpp)
ament_target_dependencies(system_state_sub
    contingency_manager_interfaces)

uas_behavior_add_bt_plugin(landing_approach_sub
    sub/landing_approach.cpp)
ament_target_dependencies(landing_approach_sub
    contingency_manager_interfaces)

uas_behavior_add_bt_plugin(landing_site_status_sub
    sub/landing_site_status.cpp)
ament_target_dependencies(landing_site_status_sub
    contingency_manager_interfaces)

uas_behavior_add_bt_plugin(contingency_event_pub
    pub/contingency_event.cpp)
ament_target_dependencies(contingency_event_pub
    contingency_manager_interfaces)

