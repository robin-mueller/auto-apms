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

#include <px4_behavior_examples/msg/landing_approach.hpp>

#include "px4_behavior/bt_ros2_node.hpp"

#define OUTPUT_KEY_SITE_ID "next_landing_site_id"

using namespace BT;
using LandingApproachMsg = px4_behavior_examples::msg::LandingApproach;

namespace px4_behavior::ops_engine {

class IsApproachingLanding : public RosTopicSubNode<LandingApproachMsg>
{
   public:
    using RosTopicSubNode::RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({OutputPort<uint8_t>(OUTPUT_KEY_SITE_ID,
                                                       "{next_landing_site_id}",
                                                       "ID of the next landing site to be approached")});
    }

    NodeStatus onTick(const std::shared_ptr<LandingApproachMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (!last_msg_ptr) {
            RCLCPP_WARN(logger(), "%s - No new landing approach message was received", name().c_str());
            return NodeStatus::FAILURE;
        }

        setOutput(OUTPUT_KEY_SITE_ID, last_msg_ptr->next_landing_site_id);

        return last_msg_ptr->is_approaching ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

}  // namespace px4_behavior::ops_engine

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ops_engine::IsApproachingLanding)
