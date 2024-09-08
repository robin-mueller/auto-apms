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

#include <px4_behavior_examples/msg/landing_site_status.hpp>

#include "px4_behavior/bt_ros2_node.hpp"

#define INPUT_KEY_SITE_ID "landing_site_id"
#define OUTPUT_KEY_SITE_STATUS "landing_site_status"

using namespace BT;
using LandingSiteStatusMsg = px4_behavior_examples::msg::LandingSiteStatus;

namespace px4_behavior::ops_engine {

class IsLandingSiteClear : public RosTopicSubNode<LandingSiteStatusMsg>
{
   public:
    using RosTopicSubNode::RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<uint8_t>(INPUT_KEY_SITE_ID, "ID of the landing site to inspect"),
             OutputPort<uint8_t>(OUTPUT_KEY_SITE_STATUS, "{landing_site_status}", "The landing status flag")});
    }

    NodeStatus onTick(const std::shared_ptr<LandingSiteStatusMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (!last_msg_ptr) { return NodeStatus::FAILURE; }

        auto return_status = BT::NodeStatus::FAILURE;
        auto status = last_msg_ptr->status[getInput<uint8_t>(INPUT_KEY_SITE_ID).value()];
        switch (status) {
            case LandingSiteStatusMsg::STATUS_CLEAR_FOR_LANDING:
                return_status = BT::NodeStatus::SUCCESS;
                break;
            case LandingSiteStatusMsg::STATUS_TEMPORARILY_BLOCKED:
                break;
            case LandingSiteStatusMsg::STATUS_PERMANENTLY_BLOCKED:
                break;
            default:
                RCLCPP_WARN(logger(), "%s: Landing status is unkown", name().c_str());
                break;
        }
        setOutput(OUTPUT_KEY_SITE_STATUS, status);
        return return_status;
    }
};

}  // namespace px4_behavior::ops_engine

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ops_engine::IsLandingSiteClear)
