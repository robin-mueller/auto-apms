// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <string>

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Additional parameters specific to ROS 2 determined at runtime by TreeBuilder.
 */
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
  template <class, typename, bool>
  friend class NodeRegistrationTemplate;

public:
  /**
   * @brief Constructor.
   *
   * @param[in] ros_node ROS 2 node instance used for adding waitables with.
   * @param[in] tree_node_waitables_callback_group Callback group to be used within tree nodes when adding waitables.
   * @param[in] tree_node_waitables_executor Executor used for executing work provided by the node's waitables.
   * @param[in] options Configuration options for registering the behavior tree node.
   */
  RosNodeContext(
    rclcpp::Node::SharedPtr ros_node, rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group,
    rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor,
    const NodeRegistrationOptions & options);

  /**
   * @brief Get the name of the ROS 2 node passed to the constructor.
   * @return Name of the associated ROS 2 node.
   */
  std::string getROSNodeName() const;

  /**
   * @brief Get the fully qualified name of the ROS 2 node passed to the constructor.
   * @return Fully qualified name of the associated ROS 2 node.
   */
  std::string getFullyQualifiedRosNodeName() const;

  /**
   * @brief Get the logger of the associated ROS 2 node.
   * @return Logger instance.
   */
  rclcpp::Logger getBaseLogger() const;

  /**
   * @brief Get a child logger created using the associated ROS 2 node.
   * @param name Name of the new logger.
   * @return Logger instance.
   */
  rclcpp::Logger getChildLogger(const std::string & name);

  /**
   * @brief Get the current time using the associated ROS 2 node.
   * @return Current time.
   */
  rclcpp::Time getCurrentTime() const;

  /**
   * @brief Create a string representing the detailed name of a behavior tree node.
   * @param node Pointer to the behavior tree node instance.
   * @param with_class_name Whether to include the name of the behavior tree node class.
   * @return Fully qualified name of @p node.
   */
  std::string getFullyQualifiedTreeNodeName(const BT::TreeNode * node, bool with_class_name = true) const;

  /**
   * @brief Get the extra YAML options provided during node registration.
   * @return YAML node with extra options.
   */
  YAML::Node getExtraOptions() const;

private:
  void modifyProvidedPortsListForRegistration(BT::PortsList & ports_list) const;

  BT::Expected<std::string> getTopicName(const BT::TreeNode * node) const;

  BT::PortsRemapping copyAliasedPortValuesToOriginalPorts(const BT::TreeNode * node) const;

  const std::string ros_node_name_;
  const std::string fully_qualified_ros_node_name_;
  rclcpp::Logger base_logger_;

  /// Handle for the ROS2 node.
  rclcpp::Node::WeakPtr nh_;
  /// Callback group to be used when adding ROS 2 waitables like subscriptions, service clients, action clients etc. do.
  rclcpp::CallbackGroup::WeakPtr cb_group_;
  /// Executor that may be used to execute work provided by the node's waitables locally.
  rclcpp::executors::SingleThreadedExecutor::WeakPtr executor_;
  /// Configuration options for registering the behavior tree node.
  const NodeRegistrationOptions registration_options_;
};

}  // namespace auto_apms_behavior_tree::core
