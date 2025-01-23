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

#include <memory>
#include <string>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/qos.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Generic behavior tree node wrapper for a ROS 2 publisher.
 *
 * When ticked, this node publishes a single message to a topic. Inheriting classes must reimplement the virtual methods
 * as described below.
 *
 * By default, the name of the topic will be determined as follows:
 *
 * 1. If a value is passed using the input port named `port`, use that.
 *
 * 2. Otherwise, use the value from NodeRegistrationOptions::port passed on construction as part of RosNodeContext.
 *
 * It is possible to customize which port is used to determine the topic name and also extend the input's value
 * with a prefix or suffix. This is achieved by including the special pattern `(input:<port_name>)` in
 * NodeRegistrationOptions::port and replacing `<port_name>` with the desired input port name.
 *
 * **Example**: Given the user implements an input port `BT::InputPort<std::string>("my_port")`, one may create a client
 * for the topic "foo/bar" by defining NodeRegistrationOptions::port as `(input:my_port)/bar` and providing the string
 * "foo" to the port with name `my_port`.
 *
 * Additionally, the following characteristics depend on NodeRegistrationOptions:
 *
 * - logger_level: Minimum severity level enabled for logging using the ROS 2 Logger API.
 *
 * @tparam MessageT Type of the ROS 2 message.
 */
template <class MessageT>
class RosPublisherNode : public BT::ConditionNode
{
  using Publisher = typename rclcpp::Publisher<MessageT>;

public:
  using MessageType = MessageT;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  /**
   * @brief Constructor.
   *
   * Derived nodes are automatically created by TreeBuilder::instantiate when included inside a node manifest
   * associated with the behavior tree resource.
   * @param instance_name Name given to this specific node instance.
   * @param config Structure of internal data determined at runtime by BT::BehaviorTreeFactory.
   * @param context Additional parameters specific to ROS 2 determined at runtime by TreeBuilder.
   * @param qos Quality of service settings forwarded to the publisher.
   */
  explicit RosPublisherNode(
    const std::string & instance_name, const Config & config, Context context, const rclcpp::QoS & qos = {10});

  virtual ~RosPublisherNode() = default;

  /**
   * @brief Derived nodes implementing the static method RosPublisherNode::providedPorts may call this method to also
   * include the default port for ROS 2 behavior tree nodes.
   *
   * @param addition Additional ports to add to the ports list.
   * @return List of ports containing the default port along with node-specific ports.
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("port", "Name of the ROS 2 topic to publish to.")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief If a behavior tree requires input/output data ports, the developer must define this method accordingly.
   * @return List of ports used by this node.
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief Callback invoked when ticked to define the message to be published.
   *
   * The node may deny to publish a message by returning `false`. Otherwise, this method should return `true`.
   *
   * By default, this callback simply returns `true` and sends an empty message.
   * @param msg Reference to the message.
   * @return `false` if no message should be published. In that case, the return status of this node will be
   * BT::NodeStatus::FAILURE. Otherwise, the message will be published and the node returns BT::NodeStatus::SUCCESS.
   */
  virtual bool setMessage(MessageT & msg);

  /**
   * @brief Create the ROS 2 publisher.
   * @param topic_name Name of the topic.
   * @return `true` if the publisher was created successfully, `false` otherwise.
   */
  bool createPublisher(const std::string & topic_name);

  /**
   * @brief Get the name of the topic name this node publishes to.
   * @return String representing the topic name.
   */
  std::string getTopicName() const;

protected:
  const Context context_;
  const rclcpp::Logger logger_;

private:
  BT::NodeStatus tick() override final;

  const rclcpp::QoS qos_;
  std::string topic_name_;
  bool dynamic_client_instance_ = false;
  std::shared_ptr<Publisher> publisher_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class MessageT>
inline RosPublisherNode<MessageT>::RosPublisherNode(
  const std::string & instance_name, const Config & config, Context context, const rclcpp::QoS & qos)
: BT::ConditionNode(instance_name, config),
  context_(context),
  logger_(context.getChildLogger(auto_apms_util::toSnakeCase(instance_name))),
  qos_{qos}
{
  if (const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this)) {
    createPublisher(expected_name.value());
  } else {
    // We assume that determining the communication port requires a blackboard pointer, which cannot be evaluated at
    // construction time. The expression will be evaluated each time before the node is ticked the first time after
    // successful execution.
    dynamic_client_instance_ = true;
  }
}

template <class MessageT>
inline bool RosPublisherNode<MessageT>::createPublisher(const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - Argument topic_name is empty when trying to create a client.");
  }

  // Check if the publisher with given name is already set up
  if (publisher_ && topic_name_ == topic_name) return true;

  rclcpp::Node::SharedPtr node = context_.nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - The weak pointer to the ROS 2 node expired. The tree node doesn't "
      "take ownership of it.");
  }

  publisher_ = node->template create_publisher<MessageT>(topic_name, qos_);
  topic_name_ = topic_name;
  RCLCPP_DEBUG(
    logger_, "%s - Created publisher for topic '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
    topic_name.c_str());
  return true;
}

template <class MessageT>
inline BT::NodeStatus RosPublisherNode<MessageT>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // If client has been set up in derived constructor, event though this constructor couldn't, we discard the intention
  // of dynamically creating the client
  if (dynamic_client_instance_ && publisher_) {
    dynamic_client_instance_ = false;
  }

  // Try again to create the client on first tick if this was not possible during construction or if client should be
  // created from a blackboard entry on the start of every iteration
  if (status() == BT::NodeStatus::IDLE && dynamic_client_instance_) {
    const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this);
    if (expected_name) {
      createPublisher(expected_name.value());
    } else {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) +
        " - Cannot create the publisher because the topic name couldn't be resolved using "
        "the communication port expression specified by the node's "
        "registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_PORT + ": " + context_.registration_options_.port +
        "). Error message: " + expected_name.error());
    }
  }

  if (!publisher_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - publisher_ is nullptr.");
  }

  MessageT msg;
  if (!setMessage(msg)) {
    return BT::NodeStatus::FAILURE;
  }
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

template <class MessageT>
inline bool RosPublisherNode<MessageT>::setMessage(MessageT & /*msg*/)
{
  return true;
}

template <class MessageT>
inline std::string RosPublisherNode<MessageT>::getTopicName() const
{
  return topic_name_;
}

}  // namespace auto_apms_behavior_tree::core