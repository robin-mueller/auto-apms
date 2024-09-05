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
#include "px4_behavior/get_resource.hpp"

#define INPUT_KEY_PACKAGE "package_name"
#define INPUT_KEY_FILENAME "filename"
#define OUTPUT_KEY_DATA "xml_data"

using namespace BT;

namespace px4_behavior {

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

        auto resource =
            FetchBehaviorTreeResource(std::filesystem::path{filename}.filename(), std::nullopt, package_name);
        if (!resource.has_value()) return NodeStatus::FAILURE;

        std::string xml_data;
        try {
            xml_data = px4_behavior::ReadBehaviorTreeFile(resource.value().tree_path);
        } catch (const std::runtime_error& e) {
            return NodeStatus::FAILURE;
        }
        setOutput<std::string>(OUTPUT_KEY_DATA, xml_data);
        return NodeStatus::SUCCESS;
    }
};

}  // namespace px4_behavior

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::LoadBehaviorTreeAction);
