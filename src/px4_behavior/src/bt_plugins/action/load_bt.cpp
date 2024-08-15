#include <px4_behavior/bt_plugins.hpp>
#include <px4_behavior/get_resource.hpp>

#define NODE_NAME "LoadBehaviorTrees"

#define INPUT_KEY_PACKAGE "package_name"
#define INPUT_KEY_FILENAME "filename"
#define OUTPUT_KEY_DATA "xml_data"

using namespace BT;

namespace contingency_manager {

class LoadBehaviorTreesAction : public SyncActionNode
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
        std::string xml_data;
        try {
            xml_data = px4_behavior::read_behavior_tree_filepath(px4_behavior::get_behavior_tree_filepath(package_name, filename));
        } catch (const std::runtime_error& e) {
            return NodeStatus::FAILURE;
        }
        setOutput<std::string>(OUTPUT_KEY_DATA, xml_data);
        return NodeStatus::SUCCESS;
    }
};

}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<contingency_manager::LoadBehaviorTreesAction>(NODE_NAME);
}
