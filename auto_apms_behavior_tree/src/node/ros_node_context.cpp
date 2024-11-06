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

#include "auto_apms_behavior_tree/node/ros_node_context.hpp"

namespace auto_apms_behavior_tree
{
RosNodeContext::RosNodeContext(rclcpp::Node::SharedPtr node_ptr, const NodeRegistrationParams & tree_node_params)
: nh(node_ptr),
  default_port_name(tree_node_params.port),
  wait_for_server_timeout(tree_node_params.wait_timeout),
  request_timeout(tree_node_params.request_timeout)
{
}

rclcpp::Logger RosNodeContext::getLogger() const
{
  if (const auto node = nh.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("RosTreeNode");
}

rclcpp::Time RosNodeContext::getCurrentTime() const
{
  if (const auto node = nh.lock()) {
    return node->now();
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

}  // namespace auto_apms_behavior_tree