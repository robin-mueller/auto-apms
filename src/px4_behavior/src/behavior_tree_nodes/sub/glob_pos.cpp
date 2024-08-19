#include <Eigen/Geometry>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_behavior/bt_ros2_node.hpp>
#include <px4_behavior/bt_px4_topic_sub_node.hpp>

#define OUTPUT_KEY_ALT "altitude"
#define OUTPUT_KEY_POS "pos_vec"

using namespace BT;
using GlobalPositionMsg = px4_msgs::msg::VehicleGlobalPosition;

namespace px4_behavior {

class ReadGlobalPosition : public PX4RosTopicSubNode<GlobalPositionMsg>
{
    GlobalPositionMsg last_msg_;

   public:
    using PX4RosTopicSubNode::PX4RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {OutputPort<double>(OUTPUT_KEY_ALT, "{altitude}", "Current altitude in meter (AMSL)"),
             OutputPort<Eigen::Vector3d>(OUTPUT_KEY_POS,
                                         "{pos_vec}",
                                         "Current global position (latitude [°], longitude [°], altitude AMSL [m])")});
    }

    NodeStatus onTick(const std::shared_ptr<GlobalPositionMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (last_msg_ptr) last_msg_ = *last_msg_ptr;

        if (auto any_locked = getLockedPortContent(OUTPUT_KEY_POS)) {
            setOutput(OUTPUT_KEY_ALT, last_msg_.alt);
            Eigen::Vector3d pos{last_msg_.lat, last_msg_.lon, last_msg_.alt};
            any_locked.assign(pos);
            return NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(logger(),
                     "%s - getLockedPortContent() failed for argument %s",
                     name().c_str(),
                     OUTPUT_KEY_POS);
        return NodeStatus::FAILURE;
    }
};

}  // namespace px4_behavior

#include <px4_behavior/register_behavior_tree_node_macro.hpp>
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ReadGlobalPosition);
