#include <px4_behavior/bt_plugins.hpp>
#include <px4_behavior_interfaces/action/bt_executor_command.hpp>

#define NODE_NAME "PauseExecutor"

using namespace BT;

namespace px4_behavior {

class PauseExecutorAction : public RosActionNode<px4_behavior_interfaces::action::BTExecutorCommand>
{
   public:
    using RosActionNode::RosActionNode;

    bool setGoal(Goal& goal)
    {
        goal.command = Goal::COMMAND_PAUSE;
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
};

}  // namespace px4_behavior

CreateRosNodePlugin(px4_behavior::PauseExecutorAction, NODE_NAME);
