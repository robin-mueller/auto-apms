#include <Eigen/Geometry>
#include <commander_interfaces/action/go_to.hpp>
#include <uas_behavior/bt_plugins.hpp>

#define NODE_NAME "GoToVector"
#define INPUT_KEY_VEC "vector"

using namespace BT;

namespace uas_behavior {

class GoToVectorAction : public RosActionNode<commander_interfaces::action::GoTo>
{
   public:
    using RosActionNode::RosActionNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<Eigen::Vector3d>(INPUT_KEY_VEC, "Target position as a pointer to a vector")});
    }

    bool setGoal(Goal& goal)
    {
        if (auto any_locked = getLockedPortContent(INPUT_KEY_VEC)) {
            if (any_locked->empty()) {
                RCLCPP_ERROR(logger(),
                             "%s - Value at blackboard entry {%s} is empty",
                             name().c_str(),
                             INPUT_KEY_VEC);
                return false;
            }
            else if (Eigen::Vector3d* vec_ptr = any_locked->castPtr<Eigen::Vector3d>()) {
                goal.lat = vec_ptr->x();
                goal.lon = vec_ptr->y();
                goal.alt = vec_ptr->z();
                goal.head_towards_destination = true;
                return true;
            }
            else {
                RCLCPP_ERROR(logger(), "%s - Failed to cast pointer {%s}", name().c_str(), INPUT_KEY_VEC);
                return false;
            }
        }
        RCLCPP_ERROR(logger(),
                     "%s - getLockedPortContent() failed for argument %s",
                     name().c_str(),
                     INPUT_KEY_VEC);
        return false;
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

CreateRosNodePlugin(uas_behavior::GoToVectorAction, NODE_NAME);
