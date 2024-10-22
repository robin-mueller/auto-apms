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
#include "auto_apms_behavior_tree/node/plugin.hpp"

#define INPUT_KEY_PACKAGE "package_name"
#define INPUT_KEY_FILENAME "filename"
#define OUTPUT_KEY_DATA "xml_data"

namespace auto_apms_behavior_tree {

class LoadBehaviorTreeAction : public BT::SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(INPUT_KEY_PACKAGE, "Name of the ROS2 package containing the trees file"),
                BT::InputPort<std::string>(INPUT_KEY_FILENAME, "Name of the trees file (Extension may be omitted)"),
                BT::OutputPort<std::string>(OUTPUT_KEY_DATA,
                                            "{xml_data}",
                                            "XML string containing the data for the behavior trees")};
    }

    BT::NodeStatus tick() final
    {
        auto package_name = getInput<std::string>(INPUT_KEY_PACKAGE).value();
        auto filename = getInput<std::string>(INPUT_KEY_FILENAME).value();

        BTCreator::SharedPtr tree_creator_ptr;
        try {
            tree_creator_ptr = BTCreator::FromTreeFileName(filename, package_name);
        } catch (const exceptions::ResourceNotFoundError& e) {
            return BT::NodeStatus::FAILURE;
        }

        setOutput<std::string>(OUTPUT_KEY_DATA, tree_creator_ptr->WriteToString());
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::LoadBehaviorTreeAction)
