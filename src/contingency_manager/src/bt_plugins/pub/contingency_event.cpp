#include <contingency_manager_interfaces/msg/contingency_event.hpp>
#include <uas_behavior/bt_plugins.hpp>

#define NODE_NAME "PublishContingencyEvent"
#define INPUT_KEY_EVENT "event"

using namespace BT;
using ContingencyEventMsg = contingency_manager_interfaces::msg::ContingencyEvent;

namespace contingency_manager {

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

}  // namespace contingency_manager

CreateRosNodePlugin(contingency_manager::ContingencyEventPubNode, NODE_NAME);
