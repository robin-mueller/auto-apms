#include <px4_behavior/bt_plugins.hpp>
#include <px4_behavior_interfaces/action/launch_bt_executor.hpp>

#define NODE_NAME "LaunchExecutor"

using namespace BT;

namespace px4_behavior {

class LaunchBTExecutorAction : public RosActionNode<px4_behavior_interfaces::action::LaunchBTExecutor>
{
    double last_running_node_timestamp_ = 0;

   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }

    bool setGoal(Goal& goal)
    {
        (void)goal;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr)
    {
        switch (wr.result->tree_result) {
            case ActionType::Result::TREE_RESULT_SUCCESS:
                return NodeStatus::SUCCESS;
            case ActionType::Result::TREE_RESULT_FAILURE:
                return NodeStatus::FAILURE;
            default:
                throw BT::RuntimeError(name() + ": Received illegal tree result " +
                                       std::to_string(wr.result->tree_result));
        }
    }

    NodeStatus onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "%s - Error: %d - %s", name().c_str(), error, toStr(error));
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        if (feedback->running_action_timestamp > last_running_node_timestamp_) {
            last_running_node_timestamp_ = feedback->running_action_timestamp;
            RCLCPP_DEBUG(logger(),
                         "%s - Tree %s is running '%s'",
                         name().c_str(),
                         feedback->root_tree_id.c_str(),
                         feedback->running_action_name.c_str());
        }
        return NodeStatus::RUNNING;
    }
};

}  // namespace px4_behavior

CreateRosNodePlugin(px4_behavior::LaunchBTExecutorAction, NODE_NAME);
