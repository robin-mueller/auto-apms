#include <px4_behavior_interfaces/action/rtl.hpp>
#include <px4_behavior/bt_plugins.hpp>

#define NODE_NAME "RTL"

using namespace BT;

namespace px4_behavior {

class RTLAction : public RosActionNode<px4_behavior_interfaces::action::RTL>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts() { return providedBasicPorts({}); }

    bool setGoal(Goal& goal)
    {
        (void)goal;
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

CreateRosNodePlugin(px4_behavior::RTLAction, NODE_NAME);
