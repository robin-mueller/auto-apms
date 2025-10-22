// Copyright 2025 Robin Müller
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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "std_srvs/srv/trigger.hpp"

#define OUTPUT_KEY_MESSAGE "message"

namespace auto_apms_behavior_tree
{

class SetTrigger : public core::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  SetTrigger(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<std::string>(OUTPUT_KEY_MESSAGE, "Service response: informational message."),
    });
  }

  bool setRequest(Request::SharedPtr & /*request*/) override final { return true; }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    setOutput(OUTPUT_KEY_MESSAGE, response->message);
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetTrigger)
