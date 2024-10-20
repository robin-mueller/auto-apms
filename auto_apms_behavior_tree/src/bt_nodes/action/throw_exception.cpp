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

#include "auto_apms_behavior_tree/node_plugin.hpp"

#define INPUT_KEY_MSG "message"

using namespace BT;

namespace auto_apms_behavior_tree {

class ThrowException : public SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static PortsList providedPorts() { return {InputPort<std::string>(INPUT_KEY_MSG, "Error message. Can be empty")}; }

    NodeStatus tick() final
    {
        auto input = getInput<std::string>(INPUT_KEY_MSG);
        auto node_name = name() == registrationName() ? registrationName() : registrationName() + ": " + name();
        if (!input.has_value()) { throw RuntimeError(node_name + " - An error occured"); }
        throw RuntimeError(node_name + " - " + input.value());
    }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::ThrowException)
