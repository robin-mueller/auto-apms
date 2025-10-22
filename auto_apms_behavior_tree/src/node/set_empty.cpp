#include "auto_apms_behavior_tree_core/node.hpp"
#include "std_srvs/srv/empty.hpp"

namespace auto_apms_behavior_tree
{

class SetEmpty : public core::RosServiceNode<std_srvs::srv::Empty>
{
public:
  SetEmpty(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  bool setRequest(Request::SharedPtr & /*request*/) override final { return true; }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & /*response*/) override final
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetEmpty)
