#include <px4_behavior_interfaces/action/takeoff.hpp>
#include <px4_behavior/bt_ros2_node.hpp>

#define INPUT_KEY_ALTITUDE "alt"

using namespace BT;

namespace px4_behavior {
    
class TakeoffAction : public RosActionNode<px4_behavior_interfaces::action::Takeoff>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<double>(INPUT_KEY_ALTITUDE, "Target takeoff altitude in meter (AMSL)")});
    }

    bool setGoal(Goal& goal)
    {
        goal.altitude_amsl_m = getInput<double>(INPUT_KEY_ALTITUDE).value();
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
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::TakeoffAction);
