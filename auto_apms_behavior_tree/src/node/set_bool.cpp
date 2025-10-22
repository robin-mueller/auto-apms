#include "std_srvs/srv/set_bool.hpp"

#include "auto_apms_behavior_tree_core/node.hpp"

#define INPUT_KEY_DATA "data"
#define OUTPUT_KEY_MESSAGE "message"

namespace auto_apms_behavior_tree
{

class SetBool : public core::RosServiceNode<std_srvs::srv::SetBool>
{
public:
  SetBool(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>(INPUT_KEY_DATA, "Boolean data to set (true, false, 1, 0)."),
      BT::OutputPort<std::string>(OUTPUT_KEY_MESSAGE, "Service response: informational message."),
    });
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<bool> expected_data = getInput<bool>(INPUT_KEY_DATA);
    if (!expected_data) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_data.error().c_str());
      return false;
    }
    request->data = expected_data.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    setOutput(OUTPUT_KEY_MESSAGE, response->message);
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetBool)
