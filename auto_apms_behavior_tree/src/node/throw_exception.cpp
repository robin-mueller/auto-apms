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

#include "auto_apms_behavior_tree_core/node.hpp"

#define INPUT_KEY_MSG "message"

namespace auto_apms_behavior_tree
{

class ThrowException : public BT::SyncActionNode
{
public:
  using SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>(INPUT_KEY_MSG, "Error message. Creates a generic error message if empty.")};
  }

  BT::NodeStatus tick() override final
  {
    auto input = getInput<std::string>(INPUT_KEY_MSG);
    auto prefix = name() == registrationName() ? name() : (name() + " (" + registrationName() + ")");
    std::string msg =
      input.has_value() ? (prefix + " - " + input.value()) : (prefix + " - Tree ran into an exception.");
    throw exceptions::RosNodeError(msg);
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(auto_apms_behavior_tree::ThrowException)
