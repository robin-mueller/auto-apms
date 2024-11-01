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

#include <auto_apms_examples/msg/landing_approach.hpp>

#include "auto_apms_behavior_tree/node.hpp"

#define OUTPUT_KEY_SITE_ID "next_landing_site_id"

using LandingApproachMsg = auto_apms_examples::msg::LandingApproach;

namespace auto_apms::ops_engine
{

class IsApproachingLanding : public auto_apms_behavior_tree::RosSubscriberNode<LandingApproachMsg>
{
public:
  using RosSubscriberNode::RosSubscriberNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::OutputPort<uint8_t>(OUTPUT_KEY_SITE_ID, "{next_landing_site_id}",
                                                        "ID of the next landing site to be approached") });
  }

  BT::NodeStatus onTick(const std::shared_ptr<LandingApproachMsg>& last_msg_ptr) override final
  {
    // Check if a new message was received
    if (!last_msg_ptr)
    {
      RCLCPP_WARN(getLogger(), "%s - No new landing approach message was received", name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    setOutput(OUTPUT_KEY_SITE_ID, last_msg_ptr->next_landing_site_id);

    return last_msg_ptr->is_approaching ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms::ops_engine

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms::ops_engine::IsApproachingLanding)
