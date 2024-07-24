#include <contingency_manager_interfaces/msg/system_state.hpp>
#include <uas_behavior/bt_plugins.hpp>

#define NODE_NAME "GetSystemState"
#define OUTPUT_KEY_BATTERY "battery_level"

using namespace BT;
using SystemStateMsg = contingency_manager_interfaces::msg::SystemState;

namespace contingency_manager {

class SystemStateSubNode : public RosTopicSubNode<SystemStateMsg>
{
    SystemStateMsg last_msg_;

   public:
    using RosTopicSubNode::RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {OutputPort<float>(OUTPUT_KEY_BATTERY, "{battery_level}", "Battery level in percent")});
    }

    NodeStatus onTick(const std::shared_ptr<SystemStateMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (last_msg_ptr) last_msg_ = *last_msg_ptr;

        setOutput(OUTPUT_KEY_BATTERY, last_msg_.battery_level_percent);
        return NodeStatus::SUCCESS;
    }
};

}  // namespace contingency_manager

CreateRosNodePlugin(contingency_manager::SystemStateSubNode, NODE_NAME);
