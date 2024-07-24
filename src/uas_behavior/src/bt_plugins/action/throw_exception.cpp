#include <uas_behavior/bt_plugins.hpp>

#define NODE_NAME "ThrowException"

#define INPUT_KEY_MSG "message"

using namespace BT;

namespace contingency_manager {

class ThrowExceptionNode : public SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static PortsList providedPorts() { return {InputPort<std::string>(INPUT_KEY_MSG, "Error message. Can be empty")}; }

    NodeStatus tick() final
    {
        auto input = getInput<std::string>(INPUT_KEY_MSG);
        auto node_name =
            name() == std::string(NODE_NAME) ? std::string(NODE_NAME) : std::string(NODE_NAME) + ": " + name();
        if (!input.has_value()) throw RuntimeError(node_name + " - An error occured");
        throw RuntimeError(node_name + " - " + input.value());
    }
};

}  // namespace contingency_manager

BT_REGISTER_NODES(factory) { factory.registerNodeType<contingency_manager::ThrowExceptionNode>(NODE_NAME); }
