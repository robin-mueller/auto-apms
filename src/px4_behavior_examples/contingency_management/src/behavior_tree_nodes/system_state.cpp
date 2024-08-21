#include <px4_behavior_examples/msg/system_state.hpp>
#include <px4_behavior/bt_ros2_node.hpp>

#define OUTPUT_KEY_BATTERY "battery_level"

using namespace BT;
using SystemStateMsg = px4_behavior_examples::msg::SystemState;

namespace px4_behavior::ops_engine {

class GetSystemState : public RosTopicSubNode<SystemStateMsg>
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

}  // namespace px4_behavior

#include <px4_behavior/register_behavior_tree_node_macro.hpp>
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ops_engine::GetSystemState)
