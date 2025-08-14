// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <optional>
#include <type_traits>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "auto_apms_util/container.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define INPUT_KEY_PARAM_VALUE "value"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

class HasParameter : public core::RosServiceNode<rcl_interfaces::srv::ListParameters>
{
public:
  HasParameter(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    if (
      config.input_ports.find(INPUT_KEY_NODE_NAME) == config.input_ports.end() ||
      config.input_ports.at(INPUT_KEY_NODE_NAME).empty()) {
      // Refer to this ROS 2 node as the target if respective input port is empty
      createClient(context_.getFullyQualifiedRosNodeName() + "/list_parameters");
    }
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the service name
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_NODE_NAME, "Name of the targeted ROS 2 node. Leave empty to target this executor's node."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter.")};
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_name = getInput<std::string>(INPUT_KEY_PARAM_NAME);
    if (!expected_name || expected_name.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - Parameter name must not be empty.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      RCLCPP_DEBUG_EXPRESSION(
        logger_, !expected_name, "%s - Error message: %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_name.error().c_str());
      return false;
    }
    has_parameter_name_ = expected_name.value();
    request->depth = Request::DEPTH_RECURSIVE;
    request->prefixes = {};
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (auto_apms_util::contains(response->result.names, has_parameter_name_)) return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string has_parameter_name_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::HasParameter)
