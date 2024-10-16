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

#include "auto_apms_behavior_tree/behavior_tree.hpp"
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "behaviortree_cpp/action_node.h"

#define INPUT_KEY_PACKAGE "package_name"
#define INPUT_KEY_FILENAME "filename"
#define OUTPUT_KEY_DATA "xml_data"

using namespace BT;

namespace auto_apms_behavior_tree {

class LoadBehaviorTreeAction : public SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>(INPUT_KEY_PACKAGE, "Name of the ROS2 package containing the trees file"),
                InputPort<std::string>(INPUT_KEY_FILENAME, "Name of the trees file (Extension may be omitted)"),
                OutputPort<std::string>(OUTPUT_KEY_DATA,
                                        "{xml_data}",
                                        "XML string containing the data for the behavior trees")};
    }

    NodeStatus tick() final
    {
        auto package_name = getInput<std::string>(INPUT_KEY_PACKAGE).value();
        auto filename = getInput<std::string>(INPUT_KEY_FILENAME).value();

        BehaviorTreeResource resource;
        try {
            resource = BehaviorTreeResource::SelectByFileName(filename, package_name);
        } catch (const exceptions::ResourceNotFoundError& e) {
            return NodeStatus::FAILURE;
        }

        setOutput<std::string>(OUTPUT_KEY_DATA, BehaviorTree{resource}.WriteToString());
        return NodeStatus::SUCCESS;
    }
};

}  // namespace auto_apms_behavior_tree

#include "auto_apms_behavior_tree/node_plugin.hpp"
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LoadBehaviorTreeAction)
