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

#include "auto_apms_behavior_tree/node/plugin.hpp"
#include "auto_apms_interfaces/action/launch_bt_executor.hpp"

namespace auto_apms_behavior_tree {

class LaunchExecutorAction : public RosActionNode<auto_apms_interfaces::action::LaunchBTExecutor>
{
    double last_running_node_timestamp_ = 0;

   public:
    using RosActionNode::RosActionNode;

    static BT::PortsList providedPorts() { return providedBasicPorts({}); }

    BT::NodeStatus onResultReceived(const WrappedResult& wr)
    {
        switch (wr.result->tree_result) {
            case ActionType::Result::TREE_RESULT_SUCCESS:
                return BT::NodeStatus::SUCCESS;
            case ActionType::Result::TREE_RESULT_FAILURE:
                return BT::NodeStatus::FAILURE;
            default:
                throw exceptions::RosNodeError(name() + ": Received illegal tree result " +
                                               std::to_string(wr.result->tree_result));
        }
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        if (feedback->running_action_timestamp > last_running_node_timestamp_) {
            last_running_node_timestamp_ = feedback->running_action_timestamp;
            RCLCPP_DEBUG(logger(),
                         "%s - Tree %s is running '%s'",
                         name().c_str(),
                         feedback->root_tree_id.c_str(),
                         feedback->running_action_name.c_str());
        }
        return BT::NodeStatus::RUNNING;
    }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LaunchExecutorAction)
