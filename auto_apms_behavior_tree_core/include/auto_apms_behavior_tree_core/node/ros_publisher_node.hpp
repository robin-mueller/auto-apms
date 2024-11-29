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
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/qos.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Abstract class to wrap a ROS publisher
 *
 */
template <class MessageT>
class RosPublisherNode : public BT::ConditionNode
{
  using Publisher = typename rclcpp::Publisher<MessageT>;

public:
  using MessageType = MessageT;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  explicit RosPublisherNode(
    const std::string & instance_name, const Config & config, const Context & context, const rclcpp::QoS & qos = {10});

  virtual ~RosPublisherNode() = default;

  /**
   * @brief Any subclass of RosPublisherNode that has additional ports must provide a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("port", "Name of the ROS 2 topic to publish to.")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  BT::NodeStatus tick() override final;

  /**
   * @brief setMessage is a callback invoked in tick to allow the user to pass
   * the message to be published.
   *
   * @param msg the message.
   * @return  return false if anything is wrong and we must not send the message.
   * the Condition will return FAILURE.
   */
  virtual bool setMessage(MessageT & msg);

  bool createPublisher(const std::string & topic_name);

  std::string getTopicName() const;

protected:
  const Context context_;

private:
  const rclcpp::Logger logger_;
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
  const std::string & instance_name, const Config & config, const Context & context, const rclcpp::QoS & qos)
: BT::ConditionNode(instance_name, config), context_(context), logger_(context.getLogger()), qos_{qos}
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
        NodeRegistrationParams::PARAM_NAME_PORT + ": " + context_.registration_params_.port +
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