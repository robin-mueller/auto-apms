#include <px4_behavior_interfaces/msg/contingency_event.hpp>
#include <px4_behavior/bt_plugins.hpp>

#define NODE_NAME "PublishContingencyEvent"
#define INPUT_KEY_EVENT "event"

using namespace BT;
using ContingencyEventMsg = px4_behavior_interfaces::msg::ContingencyEvent;

namespace px4_behavior {

class ContingencyEventPubNode : public RosTopicPubNode<ContingencyEventMsg>
{
   public:
    using RosTopicPubNode::RosTopicPubNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<uint8_t>(INPUT_KEY_EVENT, "ID of the occured contingency event")});
    }

    bool setMessage(ContingencyEventMsg& msg) final
    {
        msg.event_id = getInput<uint8_t>(INPUT_KEY_EVENT).value();
        return true;
    }
};

}  // namespace px4_behavior

CreateRosNodePlugin(px4_behavior::ContingencyEventPubNode, NODE_NAME);
