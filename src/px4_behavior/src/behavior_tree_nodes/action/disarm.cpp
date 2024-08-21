#include <px4_behavior_interfaces/action/arm_disarm.hpp>
#include <px4_behavior/bt_ros2_node.hpp>

#define NODE_NAME "Disarm"

using namespace BT;

namespace px4_behavior {

class DisarmAction : public RosActionNode<px4_behavior_interfaces::action::ArmDisarm>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts() { return providedBasicPorts({}); }

    bool setGoal(Goal& goal)
    {
        goal.arming_state = Goal::ARMING_STATE_DISARM;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr)
    {
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) return NodeStatus::SUCCESS;
        return NodeStatus::FAILURE;
    }

    NodeStatus onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "%s - Error: %d - %s", name().c_str(), error, toStr(error));
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        (void)feedback;
        return NodeStatus::RUNNING;
    }
};

}  // namespace px4_behavior

#include <px4_behavior/register_behavior_tree_node_macro.hpp>
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::DisarmAction);
