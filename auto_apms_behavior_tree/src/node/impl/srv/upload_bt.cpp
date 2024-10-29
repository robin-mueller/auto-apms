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

#include "auto_apms_behavior_tree/node.hpp"
#include "auto_apms_interfaces/srv/upload_behavior_tree.hpp"

#define INPUT_KEY_TREE "xml_data"
#define INPUT_KEY_ID "tree_id"

namespace auto_apms_behavior_tree
{

class UploadBehaviorTreeAction : public RosServiceNode<auto_apms_interfaces::srv::UploadBehaviorTree>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>(INPUT_KEY_TREE,
                                                           "XML string containing the data for the behavior trees to "
                                                           "be registered"),
                                BT::InputPort<std::string>(INPUT_KEY_ID,
                                                           "ID of the tree that should be created. Empty means to use "
                                                           "the "
                                                           "main_tree_to_execute attribute to determine which tree is "
                                                           "to be executed") });
  }

  bool setRequest(Request::SharedPtr& request) final
  {
    request->xml_data = getInput<std::string>(INPUT_KEY_TREE).value();
    request->tree_id = getInput<std::string>(INPUT_KEY_ID).value_or("");  // Empty is supported in this case
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) final
  {
    if (response->success)
    {
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(logger(), "%s - Error: %s", name().c_str(), response->error_message.c_str());
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::UploadBehaviorTreeAction)
