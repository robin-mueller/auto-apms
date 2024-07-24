#include <commander_interfaces/action/arm_disarm.hpp>
#include <px4_behavior/bt_plugins.hpp>

#define NODE_NAME "Arm"
#define INPUT_KEY_WAIT "wait_until_ready_to_arm"

using namespace BT;

namespace px4_behavior {

class ArmAction : public RosActionNode<commander_interfaces::action::ArmDisarm>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<bool>(INPUT_KEY_WAIT,
                             true,
                             "Wait for the UAV to be ready for arming. If false and UAV is not ready to arm, will "
                             "be rejected.")});
    }

    bool setGoal(Goal& goal)
    {
        goal.arming_state = Goal::ARMING_STATE_ARM;
        goal.wait_until_ready_to_arm = getInput<bool>(INPUT_KEY_WAIT).value();
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

CreateRosNodePlugin(px4_behavior::ArmAction, NODE_NAME);
