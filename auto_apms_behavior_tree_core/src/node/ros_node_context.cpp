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

#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"

#include <iterator>
#include <regex>

#include "auto_apms_behavior_tree_core/exceptions.hpp"

namespace auto_apms_behavior_tree::core
{
RosNodeContext::RosNodeContext(
  rclcpp::Node::WeakPtr ros_node, rclcpp::CallbackGroup::WeakPtr tree_node_waitables_callback_group,
  rclcpp::executors::SingleThreadedExecutor::WeakPtr tree_node_waitables_executor,
  const NodeRegistrationParams & registration_params)
: nh_(ros_node),
  cb_group_(tree_node_waitables_callback_group),
  executor_(tree_node_waitables_executor),
  registration_params_(registration_params)
{
}

std::string RosNodeContext::getROSNodeName() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->get_name();
  }
  return "unkown";
}

std::string RosNodeContext::getFullyQualifiedROSNodeName() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->get_fully_qualified_name();
  }
  return "unkown";
}

rclcpp::Logger RosNodeContext::getLogger() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("RosTreeNode");
}

rclcpp::Time RosNodeContext::getCurrentTime() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->now();
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

std::string RosNodeContext::getFullyQualifiedTreeNodeName(const BT::TreeNode * node) const
{
  // NOTE: registrationName() is empty during construction as this member is first set after the factory constructed the
  // object
  const std::string instance_name = node->name();
  const std::string registration_name = node->registrationName();
  if (registration_name.empty() || instance_name == registration_name)
    return instance_name + " (" + registration_params_.class_name + ")";
  return instance_name + " (" + registration_name + " : " + registration_params_.class_name + ")";
}

BT::Expected<std::string> RosNodeContext::getCommunicationPortName(const BT::TreeNode * node) const
{
  std::string res = registration_params_.port;
  BT::PortsRemapping input_ports = node->config().input_ports;

  // Parameter registration_params_.port may contain substrings, that that are to be replaced with values retrieved from
  // a specific node input port. Must be something like (input:my_port) where 'my_port' is the key/name of the
  // BT::InputPort to use. Anything before or after the expression is kept and used as a prefix respectively suffix.
  const std::regex pattern(R"(\(input:([^)\s]+)\))");
  const std::sregex_iterator replace_begin(res.begin(), res.end(), pattern);
  const std::sregex_iterator replace_end = std::sregex_iterator();
  for (std::sregex_iterator it = replace_begin; it != replace_end; ++it) {
    const std::smatch match = *it;
    const std::string input_port_key = match[1].str();

    // Search for the specified input port key in the list of input ports passed at construction time
    if (input_ports.find(input_port_key) != input_ports.end()) {
      // The input port has been found using input_port_key

      // Make sure its value is either a blackboard pointer or a static string (Must not be empty)
      if (input_ports.at(input_port_key).empty()) {
        throw exceptions::RosNodeError(
          getFullyQualifiedTreeNodeName(node) +
          " - Cannot get the name of the node's ROS 2 communication port: Node input port '" + input_port_key +
          "' required by substring '" + match.str() + "' must not be empty.");
      }

      // We try to get the value from the node input port. If the value is a string pointing at a blackboard entry, this
      // does not work during construction time. In that case we return the unexpected value to indicate we must try
      // again.
      const BT::Expected<std::string> expected = node->getInput<std::string>(input_port_key);
      if (expected) {
        // Replace the input pattern with the value returned from getInput()
        res.replace(match.position(), match.length(), expected.value());
      } else {
        // Return unexpected if input is unavailable
        return expected;
      }
    } else {
      throw exceptions::RosNodeError(
        getFullyQualifiedTreeNodeName(node) +
        " - Cannot get the name of the node's ROS 2 communication port: Node input port '" + input_port_key +
        "' required by substring '" + match.str() + "' doesn't exist.");
    }
  }
  return res;
}

}  // namespace auto_apms_behavior_tree::core