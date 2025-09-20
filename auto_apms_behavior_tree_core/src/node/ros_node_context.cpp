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
#include "auto_apms_util/logging.hpp"

namespace auto_apms_behavior_tree::core
{
RosNodeContext::RosNodeContext(
  rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
  const NodeRegistrationOptions & options)
: ros_node_name_(ros_node ? ros_node->get_name() : ""),
  fully_qualified_ros_node_name_(ros_node ? ros_node->get_fully_qualified_name() : ""),
  base_logger_(ros_node ? ros_node->get_logger() : rclcpp::get_logger("")),
  nh_(ros_node),
  cb_group_(tree_node_waitables_callback_group),
  executor_(tree_node_waitables_executor),
  registration_options_(options)
{
}

std::string RosNodeContext::getROSNodeName() const { return ros_node_name_; }

std::string RosNodeContext::getFullyQualifiedRosNodeName() const { return fully_qualified_ros_node_name_; }

rclcpp::Logger RosNodeContext::getBaseLogger() const { return base_logger_; }

rclcpp::Logger RosNodeContext::getChildLogger(const std::string & name)
{
  const rclcpp::Logger child_logger = base_logger_.get_child(name);
  if (!registration_options_.logger_level.empty()) {
    try {
      auto_apms_util::setLoggingSeverity(child_logger, registration_options_.logger_level);
    } catch (const auto_apms_util::exceptions::SetLoggingSeverityError & e) {
      RCLCPP_ERROR(
        base_logger_,
        "Failed to set the logging severity for the child logger using the node's registration options: %s", e.what());
    }
  }
  return child_logger;
}

rclcpp::Time RosNodeContext::getCurrentTime() const
{
  if (const rclcpp::Node::SharedPtr node = nh_.lock()) {
    return node->now();
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

std::string RosNodeContext::getFullyQualifiedTreeNodeName(const BT::TreeNode * node, bool with_class_name) const
{
  // NOTE: registrationName() is empty during construction as this member is first set after the factory constructed the
  // object
  const std::string instance_name = node->name();
  const std::string registration_name = node->registrationName();
  if (!registration_name.empty() && instance_name != registration_name) {
    if (with_class_name) {
      return instance_name + " (" + registration_name + " : " + registration_options_.class_name + ")";
    }
    return instance_name + " (" + registration_name + ")";
  }
  return with_class_name ? (instance_name + " (" + registration_options_.class_name + ")") : instance_name;
}

BT::Expected<std::string> RosNodeContext::getCommunicationPortName(const BT::TreeNode * node) const
{
  std::string res = registration_options_.connection;
  BT::PortsRemapping input_ports = node->config().input_ports;

  // Parameter registration_options_.port may contain substrings, that that are to be replaced with values retrieved
  // from a specific node input port. Must be something like (input:my_port) where 'my_port' is the key/name of the
  // BT::InputPort to use. Anything before or after the expression is kept and used as a prefix respectively suffix.
  const std::regex pattern(R"(\(input:([^)\s]+)\))");
  const std::sregex_iterator replace_begin(res.begin(), res.end(), pattern);
  const std::sregex_iterator replace_end = std::sregex_iterator();

  // We iterate over each substitution expression. If there are none, this for loop has no effect, and we simply return
  // the parameters value.
  for (std::sregex_iterator it = replace_begin; it != replace_end; ++it) {
    const std::smatch match = *it;
    const std::string input_port_key = match[1].str();

    // Search for the specified input port key in the list of input ports passed at construction time
    if (input_ports.find(input_port_key) != input_ports.end()) {
      // The input port has been found using input_port_key

      // Make sure its value is either a blackboard pointer or a static string (Must not be empty)
      if (input_ports.at(input_port_key).empty()) {
        return nonstd::make_unexpected(
          getFullyQualifiedTreeNodeName(node) +
          " - Cannot get the name of the node's ROS 2 communication port: Input port '" + input_port_key +
          "' required by substring '" + match.str() + "' must not be empty.");
      }

      // We try to get the value from the node input port. If the value is a string pointing at a blackboard entry, this
      // may not work during construction time. In case the expected value contains an error, we forward it to indicate
      // we weren't successful.
      const BT::Expected<std::string> expected = node->getInput<std::string>(input_port_key);
      if (expected) {
        // Replace the respective substring with the value returned from getInput()
        res.replace(match.position(), match.length(), expected.value());
      } else {
        // Return expected (contains error) if value couldn't be retrieved from input ports
        return expected;
      }
    } else {
      return nonstd::make_unexpected(
        getFullyQualifiedTreeNodeName(node) +
        " - Cannot get the name of the node's ROS 2 communication port: Node input port '" + input_port_key +
        "' required by substring '" + match.str() + "' doesn't exist.");
    }
  }
  return res;
}

}  // namespace auto_apms_behavior_tree::core