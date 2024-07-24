#include <commander_interfaces/action/takeoff.hpp>
#include <uas_behavior/bt_plugins.hpp>

#define NODE_NAME "Takeoff"
#define INPUT_KEY_ALTITUDE "alt"

using namespace BT;

namespace uas_behavior {
    
class TakeoffAction : public RosActionNode<commander_interfaces::action::Takeoff>
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

}  // namespace uas_behavior

CreateRosNodePlugin(uas_behavior::TakeoffAction, NODE_NAME);
