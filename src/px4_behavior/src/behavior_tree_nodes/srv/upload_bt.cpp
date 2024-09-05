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
#include "px4_behavior_interfaces/srv/upload_behavior_tree.hpp"

#define INPUT_KEY_TREE "xml_data"
#define INPUT_KEY_ID "tree_id"

using namespace BT;

namespace px4_behavior {

class UploadBehaviorTreeAction : public RosServiceNode<px4_behavior_interfaces::srv::UploadBehaviorTree>
{
   public:
    using RosServiceNode::RosServiceNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<std::string>(INPUT_KEY_TREE,
                                    "XML string containing the data for the behavior trees to be registered"),
             InputPort<std::string>(INPUT_KEY_ID,
                                    "ID of the tree that should be created. Empty means to use the "
                                    "main_tree_to_execute attribute to determine which tree is to be executed")});
    }

    bool setRequest(Request::SharedPtr& request) final
    {
        request->xml_data = getInput<std::string>(INPUT_KEY_TREE).value();
        request->tree_id = getInput<std::string>(INPUT_KEY_ID).value_or("");  // Empty is supported in this case
        return true;
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) final
    {
        if (response->success) { return NodeStatus::SUCCESS; }
        RCLCPP_ERROR(logger(), "%s - Error: %s", name().c_str(), response->error_message.c_str());
        return NodeStatus::FAILURE;
    }
};

}  // namespace px4_behavior

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::UploadBehaviorTreeAction);
