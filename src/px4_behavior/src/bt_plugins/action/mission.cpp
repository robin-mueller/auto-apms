#include <px4_behavior_interfaces/action/mission.hpp>
#include <px4_behavior/bt_plugins.hpp>

#define NODE_NAME "Mission"
#define INPUT_KEY_DO_RESTART "do_restart"

using namespace BT;

namespace px4_behavior {

class MissionAction : public RosActionNode<px4_behavior_interfaces::action::Mission>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<bool>(INPUT_KEY_DO_RESTART, false, "Wether to restart (true) or resume (false) the mission.")});
    }

    bool setGoal(Goal& goal)
    {
        goal.do_restart = getInput<bool>(INPUT_KEY_DO_RESTART).value();
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

CreateRosNodePlugin(px4_behavior::MissionAction, NODE_NAME);
