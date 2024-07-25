px4_behavior_add_bt_plugin(throw_exception_action 
    action/throw_exception.cpp)

px4_behavior_add_bt_plugin(arm_action 
    action/arm.cpp)
ament_target_dependencies(arm_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(disarm_action 
    action/disarm.cpp)
ament_target_dependencies(disarm_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(enable_hold_action 
    action/enable_hold.cpp)
ament_target_dependencies(enable_hold_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(goto_action 
    action/goto.cpp)
ament_target_dependencies(goto_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(goto_vector_action 
    action/goto_vec.cpp)
ament_target_dependencies(goto_vector_action
    px4_behavior_interfaces 
    Eigen3)

px4_behavior_add_bt_plugin(land_action 
    action/land.cpp)
ament_target_dependencies(land_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(rtl_action 
    action/rtl.cpp)
ament_target_dependencies(rtl_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(takeoff_action 
    action/takeoff.cpp)
ament_target_dependencies(takeoff_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(mission_action 
    action/mission.cpp)
ament_target_dependencies(mission_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(launch_executor_action 
    action/launch_executor.cpp)
ament_target_dependencies(launch_executor_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(pause_executor_action 
    action/pause_executor.cpp)
ament_target_dependencies(pause_executor_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(resume_executor_action 
    action/resume_executor.cpp)
ament_target_dependencies(resume_executor_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(halt_executor_action 
    action/halt_executor.cpp)
ament_target_dependencies(halt_executor_action
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(load_bt_action 
    action/load_bt.cpp)
ament_target_dependencies(load_bt_action
    px4_behavior_interfaces)
target_include_directories(load_bt_action PRIVATE ${CMAKE_SOURCE_DIR}/include)

px4_behavior_add_bt_plugin(upload_bt_srv 
    srv/upload_bt.cpp)
ament_target_dependencies(upload_bt_srv
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(glob_pos_sub
    sub/glob_pos.cpp)
ament_target_dependencies(glob_pos_sub
    px4_msgs
    Eigen3)

    px4_behavior_add_bt_plugin(async_precondition
    control/async_precondition.cpp)

px4_behavior_add_bt_plugin(create_alt_land_mission_action
    action/create_alternate_landing_mission.cpp)
ament_target_dependencies(create_alt_land_mission_action
    Eigen3
    px4_ros2_cpp)

px4_behavior_add_bt_plugin(system_state_sub
    sub/system_state.cpp)
ament_target_dependencies(system_state_sub
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(landing_approach_sub
    sub/landing_approach.cpp)
ament_target_dependencies(landing_approach_sub
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(landing_site_status_sub
    sub/landing_site_status.cpp)
ament_target_dependencies(landing_site_status_sub
    px4_behavior_interfaces)

px4_behavior_add_bt_plugin(contingency_event_pub
    pub/contingency_event.cpp)
ament_target_dependencies(contingency_event_pub
    px4_behavior_interfaces)
