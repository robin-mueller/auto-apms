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

#pragma once

#include <chrono>
#include <string>

#include "auto_apms_behavior_tree_core/node/node_registration_params.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree::core
{

class RosNodeContext
{
  template <typename>
  friend class RosActionNode;
  template <typename>
  friend class RosServiceNode;
  template <typename>
  friend class RosSubscriberNode;
  template <typename>
  friend class RosPublisherNode;

public:
  RosNodeContext(
    rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
    const NodeRegistrationParams & registration_params);

  std::string getROSNodeName() const;

  std::string getFullyQualifiedRosNodeName() const;

  rclcpp::Logger getLogger() const;

  rclcpp::Time getCurrentTime() const;

  std::string getFullyQualifiedTreeNodeName(const BT::TreeNode * node, bool with_class_name = true) const;

private:
  BT::Expected<std::string> getCommunicationPortName(const BT::TreeNode * node) const;

  const std::string ros_node_name_;
  const std::string fully_qualified_ros_node_name_;
  const rclcpp::Logger logger_;

  /// Handle for the ROS2 node.
  std::weak_ptr<rclcpp::Node> nh_;
  /// Callback group to be used when adding ROS 2 waitables like subscriptions, service clients, action clients etc. do.
  rclcpp::CallbackGroup::WeakPtr cb_group_;
  /// Executor that may be used to execute work provided by the node's waitables locally.
  rclcpp::executors::SingleThreadedExecutor::WeakPtr executor_;
  /// Node specific registration parameters.
  const NodeRegistrationParams registration_params_;
};

}  // namespace auto_apms_behavior_tree::core
