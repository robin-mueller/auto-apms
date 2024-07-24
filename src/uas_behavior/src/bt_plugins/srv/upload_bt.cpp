#include <uas_behavior/bt_plugins.hpp>
#include <uas_behavior_interfaces/srv/upload_behavior_tree.hpp>

#define NODE_NAME "UploadBehaviorTree"
#define INPUT_KEY_TREE "xml_data"
#define INPUT_KEY_ID "tree_id"

using namespace BT;

namespace uas_behavior {

class UploadBehaviorTreeAction : public RosServiceNode<uas_behavior_interfaces::srv::UploadBehaviorTree>
{
   public:
    using RosServiceNode::RosServiceNode;

    static PortsList providedPorts()
    {
        return providedBasicPorts(
            {InputPort<std::string>(INPUT_KEY_TREE,
                                    "XML string containing the data for the behavior trees to be registered"),
             InputPort<std::string>(INPUT_KEY_ID,
                                    "ID of the tree that should be created. Empty means to use the "
                                    "main_tree_to_execute attribute to determine which tree is to be executed")});
    }

    bool setRequest(Request::SharedPtr& request) final
    {
        request->xml_data = getInput<std::string>(INPUT_KEY_TREE).value();
        request->tree_id = getInput<std::string>(INPUT_KEY_ID).value_or("");  // Empty is supported in this case
        return true;
    }

    NodeStatus onResponseReceived(const Response::SharedPtr& response) final
    {
        if (response->success) { return NodeStatus::SUCCESS; }
        RCLCPP_ERROR(logger(), "%s - Error: %s", name().c_str(), response->error_message.c_str());
        return NodeStatus::FAILURE;
    }
};

}  // namespace uas_behavior

CreateRosNodePlugin(uas_behavior::UploadBehaviorTreeAction, NODE_NAME);
