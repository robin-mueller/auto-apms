#pragma once

#include <px4_behavior/factory.hpp>
#include <px4_behavior_interfaces/msg/contingency_event.hpp>

using ContingencyEventMsg = px4_behavior_interfaces::msg::ContingencyEvent;

namespace px4_behavior {

// Task names (ROS 2 actions)
const char ARM_DISARM_MANEUVER_NAME[] = "arm_disarm";
const char ENABLE_HOLD_MANEUVER_NAME[] = "enable_hold";
const char GO_TO_MANEUVER_NAME[] = "go_to";
const char LAND_MANEUVER_NAME[] = "land";
const char RTL_MANEUVER_NAME[] = "rtl";
const char TAKEOFF_MANEUVER_NAME[] = "takeoff";
const char MISSION_MANEUVER_NAME[] = "mission";

const char BT_EXECUTOR_UPLOAD_SERVICE_NAME_SUFFIX[] = "/upload";
const char BT_EXECUTOR_LAUNCH_ACTION_NAME_SUFFIX[] = "/run";
const char BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX[] = "/command";

const char CONTINGENCY_EVENT_TOPIC_NAME[] = "contingency_event";
const char FORCE_CONTINGENCY_TOPIC_NAME[] = "force_contingency";
const char SYSTEM_STATE_TOPIC_NAME[] = "system_state";
const char LANDING_SITE_STATUS_TOPIC_NAME[] = "landing_site_status";
const char LANDING_APPROCH_TOPIC_NAME[] = "landing_approach";

inline std::string to_string(ContingencyEventMsg msg)
{
    switch (msg.event_id) {
        case ContingencyEventMsg::NO_EVENT:
            return "NO_EVENT";
        case ContingencyEventMsg::EVENT_BATTERY_CRITICAL:
            return "EVENT_BATTERY_CRITICAL";
        case ContingencyEventMsg::EVENT_LANDING_TEMP_BLOCKED:
            return "EVENT_LANDING_TEMP_BLOCKED";
        case ContingencyEventMsg::EVENT_LANDING_PERM_BLOCKED:
            return "EVENT_LANDING_PERM_BLOCKED";
        case ContingencyEventMsg::EVENT_UNDEFINED:
            return "EVENT_UNDEFINED";
        default:
            break;
    }
    return "undefined";
}

inline void RegisterContingencyEventEnum(BT::BehaviorTreeFactory& factory)
{
    ContingencyEventMsg event_msg;
    for (int i = 0; i <= UINT8_MAX; i++) {
        event_msg.event_id = i;
        if (auto str = to_string(event_msg); str != "undefined") {
            factory.registerScriptingEnum(str, event_msg.event_id);
        }
    }
}

}  // namespace px4_behavior
