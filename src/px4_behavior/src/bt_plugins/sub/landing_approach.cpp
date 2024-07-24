#include <px4_behavior_interfaces/msg/landing_approach.hpp>
#include <px4_behavior/bt_plugins.hpp>

#define NODE_NAME "IsApproachingLanding"
#define OUTPUT_KEY_SITE_ID "next_landing_site_id"

using namespace BT;
using LandingApproachMsg = px4_behavior_interfaces::msg::LandingApproach;

namespace px4_behavior {

class LandingApproachSubNode : public RosTopicSubNode<LandingApproachMsg>
{
   public:
    using RosTopicSubNode::RosTopicSubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({OutputPort<uint8_t>(OUTPUT_KEY_SITE_ID,
                                                   "{next_landing_site_id}",
                                                   "ID of the next landing site to be approached")});
    }

    NodeStatus onTick(const std::shared_ptr<LandingApproachMsg>& last_msg_ptr) final
    {
        // Check if a new message was received
        if (!last_msg_ptr) {
            RCLCPP_WARN(logger(), "%s - No new landing approach message was received", name().c_str());
            return NodeStatus::FAILURE;
        }

        setOutput(OUTPUT_KEY_SITE_ID, last_msg_ptr->next_landing_site_id);

        return last_msg_ptr->is_approaching ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

}  // namespace px4_behavior

CreateRosNodePlugin(px4_behavior::LandingApproachSubNode, NODE_NAME);
