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

#include "auto_apms_behavior_tree/node/node_registration_params.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

struct RosNodeContext
{
  RosNodeContext(rclcpp::Node::SharedPtr node_ptr, const NodeRegistrationParams & tree_node_params);

  /// Handle for the ROS2 node.
  std::weak_ptr<rclcpp::Node> nh;
  /**
   * @brief Default port name of the corresponding ROS 2 communication interface.
   *
   * This has different meaning based on the context:
   * - RosActionNode: Name of the action server
   * - RosServiceNode: Name of the service
   * - RosPublisherNode: Name of the topic to publish to
   * - RosSubscriberNode: Name of the topic to subscribe to
   */
  std::string default_port_name;
  /// Timeout [s] for initially discovering the associated ROS2 node.
  std::chrono::duration<double> wait_for_server_timeout;
  /// Timeout [s] for waiting for a response for the requested service or goal.
  std::chrono::duration<double> request_timeout;

  rclcpp::Logger getLogger() const;

  rclcpp::Time getCurrentTime() const;
};

}  // namespace auto_apms_behavior_tree
