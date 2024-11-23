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

namespace auto_apms_behavior_tree::core
{
RosNodeContext::RosNodeContext(rclcpp::Node::SharedPtr node_ptr, const NodeRegistrationParams & registration_params)
: nh_(node_ptr), registration_params_(registration_params)
{
}

rclcpp::Logger RosNodeContext::getLogger() const
{
  if (const auto node = nh_.lock()) {
    return node->get_logger();
  }
  return rclcpp::get_logger("RosTreeNode");
}

rclcpp::Time RosNodeContext::getCurrentTime() const
{
  if (const auto node = nh_.lock()) {
    return node->now();
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

std::string RosNodeContext::getFullName(const BT::TreeNode * node) const
{
  // NOTE: registrationName() is empty during construction as this member is frist set after the factory constructed the
  // object
  const std::string instance_name = node->name();
  const std::string registration_name = node->registrationName();
  if (registration_name.empty() || instance_name == registration_name)
    return instance_name + " (" + registration_params_.class_name + ")";
  return instance_name + " (" + registration_name + " : " + registration_params_.class_name + ")";
}

}  // namespace auto_apms_behavior_tree::core