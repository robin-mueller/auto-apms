// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "px4_behavior/bt_ros2_node.hpp"
#include "px4_behavior_interfaces/action/launch_bt_executor.hpp"

using namespace BT;

namespace px4_behavior {

class LaunchExecutorAction : public RosActionNode<px4_behavior_interfaces::action::LaunchBTExecutor>
{
    double last_running_node_timestamp_ = 0;

   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts() { return providedBasicPorts({}); }

    bool setGoal(Goal& goal)
    {
        (void)goal;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr)
    {
        switch (wr.result->tree_result) {
            case ActionType::Result::TREE_RESULT_SUCCESS:
                return NodeStatus::SUCCESS;
            case ActionType::Result::TREE_RESULT_FAILURE:
                return NodeStatus::FAILURE;
            default:
                throw BT::RuntimeError(name() + ": Received illegal tree result " +
                                       std::to_string(wr.result->tree_result));
        }
    }

    NodeStatus onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "%s - Error: %d - %s", name().c_str(), error, toStr(error));
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        if (feedback->running_action_timestamp > last_running_node_timestamp_) {
            last_running_node_timestamp_ = feedback->running_action_timestamp;
            RCLCPP_DEBUG(logger(),
                         "%s - Tree %s is running '%s'",
                         name().c_str(),
                         feedback->root_tree_id.c_str(),
                         feedback->running_action_name.c_str());
        }
        return NodeStatus::RUNNING;
    }
};

}  // namespace px4_behavior

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::LaunchExecutorAction);
