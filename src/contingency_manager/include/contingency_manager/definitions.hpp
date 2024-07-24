#pragma once

#include <contingency_manager_interfaces/msg/contingency_event.hpp>
#include <uas_behavior/factory.hpp>

using ContingencyEventMsg = contingency_manager_interfaces::msg::ContingencyEvent;

namespace contingency_manager {

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

}  // namespace contingency_manager