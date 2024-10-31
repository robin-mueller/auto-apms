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

#include <auto_apms_behavior_tree/bt_executor.hpp>
#include <auto_apms_examples/msg/contingency_event.hpp>
#include <definitions.hpp>

#define KEY_EVENT_ID "event_id"
#define KEY_EVENT_NAME "event_name"

using namespace auto_apms_behavior_tree;
using ContingencyEventMsg = auto_apms_examples::msg::ContingencyEvent;

namespace auto_apms::ops_engine
{

class ContingencyManagerExecutor : public BTExecutor
{
public:
  ContingencyManagerExecutor(const rclcpp::NodeOptions& options);

private:
  void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final;
  Command ReviewControlCommand(Command current_command, State current_state) final;
  void BeforeFirstTick(BT::Blackboard&) final;
  ClosureConduct OnResult(bool success) final;

  rclcpp::Subscription<ContingencyEventMsg>::SharedPtr sub_contingency_event_ptr_;
  bool critical_event_detected_{ false };
};

ContingencyManagerExecutor::ContingencyManagerExecutor(const rclcpp::NodeOptions& options)
  : BTExecutor{ "contingency_manager", options, 5555 }
{
  sub_contingency_event_ptr_ = node()->create_subscription<ContingencyEventMsg>(
      CONTINGENCY_EVENT_TOPIC_NAME, 10, [this](std::unique_ptr<ContingencyEventMsg> msg) {
        // Publish event information to tree
        global_blackboard()->set<uint8_t>(KEY_EVENT_ID, msg->event_id);
        global_blackboard()->set<std::string>(KEY_EVENT_NAME, toStr(*msg));

        // Set flag to wake up the contingency manager if a critical event has been detected
        critical_event_detected_ = false;
        if (msg->event_id == ContingencyEventMsg::NO_EVENT)
        {
          return;
        }
        if (msg->event_id == ContingencyEventMsg::EVENT_UNDEFINED)
        {
          RCLCPP_WARN(node()->get_logger(), "Received an undefined contingency event");
          return;
        }
        critical_event_detected_ = true;
      });
}

void ContingencyManagerExecutor::SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr,
                                                          BT::BehaviorTreeFactory& factory)
{
  // Enums (don't rely on magic enums for error safety)
  RegisterContingencyEventEnum(factory);
}

ContingencyManagerExecutor::Command ContingencyManagerExecutor::ReviewControlCommand(Command current_command,
                                                                                     State current_state)
{
  // Leave executor in IDLE until it needs to wake up
  if (!critical_event_detected_ && current_state == State::IDLE && current_command == Command::RUN)
  {
    return Command::PAUSE;
  }

  return current_command;
}

void ContingencyManagerExecutor::BeforeFirstTick(BT::Blackboard& global_blackboard)
{
  RCLCPP_INFO(node()->get_logger(), "Contingency Manager wakes up due to event %s",
              global_blackboard.get<std::string>(KEY_EVENT_NAME).c_str());
}

ContingencyManagerExecutor::ClosureConduct ContingencyManagerExecutor::OnResult(bool success)
{
  if (success)
  {
    return ClosureConduct::RESTART;
  }
  return ClosureConduct::ABORT;
}

}  // namespace auto_apms::ops_engine

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms::ops_engine::ContingencyManagerExecutor);
