// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "auto_apms_examples/msg/contingency_event.hpp"
#include "behaviortree_cpp/bt_factory.h"

using ContingencyEventMsg = auto_apms_examples::msg::ContingencyEvent;

namespace auto_apms::ops_engine
{

const char CONTINGENCY_EVENT_TOPIC_NAME[] = "contingency_event";
const char FORCE_CONTINGENCY_TOPIC_NAME[] = "force_contingency";
const char SYSTEM_STATE_TOPIC_NAME[] = "system_state";
const char LANDING_SITE_STATUS_TOPIC_NAME[] = "landing_site_status";
const char LANDING_APPROCH_TOPIC_NAME[] = "landing_approach";

inline std::string toStr(ContingencyEventMsg msg)
{
  switch (msg.event_id)
  {
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
  for (int i = 0; i <= UINT8_MAX; i++)
  {
    event_msg.event_id = i;
    if (auto str = toStr(event_msg); str != "undefined")
    {
      factory.registerScriptingEnum(str, event_msg.event_id);
    }
  }
}

}  // namespace auto_apms::ops_engine
