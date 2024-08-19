#include <px4_behavior/bt_ros2_node.hpp>

#define INPUT_KEY_MSG "message"

using namespace BT;

namespace px4_behavior {

class ThrowException : public SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static PortsList providedPorts() { return {InputPort<std::string>(INPUT_KEY_MSG, "Error message. Can be empty")}; }

    NodeStatus tick() final
    {
        auto input = getInput<std::string>(INPUT_KEY_MSG);
        auto node_name =
            name() == registrationName() ? registrationName() : registrationName() + ": " + name();
        if (!input.has_value()) throw RuntimeError(node_name + " - An error occured");
        throw RuntimeError(node_name + " - " + input.value());
    }
};

}  // namespace px4_behavior

#include <px4_behavior/register_behavior_tree_node_macro.hpp>
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ThrowException);
