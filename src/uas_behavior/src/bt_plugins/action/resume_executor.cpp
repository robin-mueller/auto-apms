#include <uas_behavior/bt_plugins.hpp>
#include <uas_behavior_interfaces/action/bt_executor_command.hpp>

#define NODE_NAME "ResumeExecutor"

using namespace BT;

namespace uas_behavior {

class ResumeExecutorAction : public RosActionNode<uas_behavior_interfaces::action::BTExecutorCommand>
{
   public:
    using RosActionNode::RosActionNode;

    bool setGoal(Goal& goal)
    {
        goal.command = Goal::COMMAND_RESUME;
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

}  // namespace uas_behavior

CreateRosNodePlugin(uas_behavior::ResumeExecutorAction, NODE_NAME);
