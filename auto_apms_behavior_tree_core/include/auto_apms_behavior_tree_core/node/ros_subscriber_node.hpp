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

#include <boost/signals2.hpp>
#include <memory>
#include <string>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/qos.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Abstract class to wrap a Topic subscriber.
 * Considering the example in the tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 *
 * The corresponding wrapper would be:
 *
 * class SubscriberNode: RosSubscriberNode<std_msgs::msg::String>
 *
 * The name of the topic will be determined as follows:
 *
 * 1. If a value is passes in the BT::InputPort "topic_name", use that
 * 2. Otherwise, use the value in RosNodeContext::default_port_name
 */
template <class MessageT>
class RosSubscriberNode : public BT::ConditionNode
{
  using Subscriber = typename rclcpp::Subscription<MessageT>;

  struct SubscriberInstance
  {
    SubscriberInstance(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name,
      const rclcpp::QoS & qos);

    std::shared_ptr<Subscriber> subscriber;
    boost::signals2::signal<void(const std::shared_ptr<MessageT>)> broadcaster;
    std::shared_ptr<MessageT> last_msg;
    std::string name;
  };

  using SubscribersRegistry = std::unordered_map<std::string, std::weak_ptr<SubscriberInstance>>;

public:
  using MessageType = MessageT;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  explicit RosSubscriberNode(
    const std::string & instance_name, const Config & config, const Context & context, const rclcpp::QoS & qos = {10});

  virtual ~RosSubscriberNode() { signal_connection_.disconnect(); }

  /**
   * @brief Any subclass of RosTopicNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("port", "Name of the ROS 2 topic to subscribe to.")};
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
   * @brief Callback invoked on every tick. You must return either SUCCESS or FAILURE.
   *
   * By default, this function calls RosSubscriberNode::onMessageReceived if a new message was received, otherwise it
   * immediately returns FAILURE.
   *
   * @param last_msg_ptr The latest message received, since the last tick. Will be `nullptr` if no new message was
   * received.
   * @return Status of the node, based on @p last_msg_ptr.
   */
  virtual BT::NodeStatus onTick(const std::shared_ptr<MessageT> & last_msg_ptr);

  /**
   * @brief Callback invoked when a new message is received. You must return either SUCCESS or FAILURE.
   *
   * This callback won't be invoked if no new message has been received since the last time it was called. If you need
   * to evaluate an expression every time this node is ticked, refer to RosSubscriberNode::onTick instead.
   *
   * @param msg Most recently received message.
   * @return Status of the node based on @p msg.
   */
  virtual BT::NodeStatus onMessageReceived(const MessageT & msg);

  bool createSubscriber(const std::string & topic_name);

  std::string getTopicName() const;

protected:
  const Context context_;

private:
  static std::mutex & registryMutex();

  // contains the fully-qualified name of the node and the name of the topic
  static SubscribersRegistry & getRegistry();

  const rclcpp::Logger logger_;
  const rclcpp::QoS qos_;
  std::string topic_name_;
  bool dynamic_client_instance_ = false;
  std::shared_ptr<SubscriberInstance> sub_instance_;
  std::shared_ptr<MessageT> last_msg_;
  boost::signals2::connection signal_connection_;
  std::string subscriber_key_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class MessageT>
inline RosSubscriberNode<MessageT>::SubscriberInstance::SubscriberInstance(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & topic_name,
  const rclcpp::QoS & qos)
{
  rclcpp::SubscriptionOptions option;
  option.callback_group = group;

  // The callback will broadcast to all the instances of RosSubscriberNode<MessageT>
  auto callback = [this](const std::shared_ptr<MessageT> msg) {
    this->last_msg = msg;
    this->broadcaster(msg);
  };
  subscriber = node->create_subscription<MessageT>(topic_name, qos, callback, option);
  name = topic_name;
}

template <class MessageT>
inline RosSubscriberNode<MessageT>::RosSubscriberNode(
  const std::string & instance_name, const Config & config, const Context & context, const rclcpp::QoS & qos)
: BT::ConditionNode{instance_name, config}, context_{context}, logger_(context.getLogger()), qos_{qos}
{
  if (const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this)) {
    createSubscriber(expected_name.value());
  } else {
    // We assume that determining the communication port requires a blackboard pointer, which cannot be evaluated at
    // construction time. The expression will be evaluated each time before the node is ticked the first time after
    // successful execution.
    dynamic_client_instance_ = true;
  }
}

template <class MessageT>
inline bool RosSubscriberNode<MessageT>::createSubscriber(const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - Argument topic_name is empty when trying to create a client.");
  }

  // Check if the subscriber with given name is already set up
  if (sub_instance_ && topic_name == sub_instance_->name) return true;

  std::unique_lock lk(registryMutex());

  rclcpp::Node::SharedPtr node = context_.nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - The weak pointer to the ROS 2 node expired. The tree node doesn't "
      "take ownership of it.");
  }
  rclcpp::CallbackGroup::SharedPtr group = context_.cb_group_.lock();
  if (!group) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - The weak pointer to the ROS 2 callback group expired. The tree node doesn't "
      "take ownership of it.");
  }
  subscriber_key_ = std::string(node->get_fully_qualified_name()) + "/" + topic_name;

  auto & registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if (it == registry.end() || it->second.expired()) {
    sub_instance_ = std::make_shared<SubscriberInstance>(node, group, topic_name, qos_);
    registry.insert({subscriber_key_, sub_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created subscriber for topic '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      topic_name.c_str());
  } else {
    sub_instance_ = it->second.lock();
  }

  // Check if there was a message received before the creation of this subscriber action
  if (sub_instance_->last_msg) {
    last_msg_ = sub_instance_->last_msg;
  }

  // add "this" as received of the broadcaster
  signal_connection_ =
    sub_instance_->broadcaster.connect([this](const std::shared_ptr<MessageT> msg) { last_msg_ = msg; });

  return true;
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // If client has been set up in derived constructor, event though this constructor couldn't, we discard the intention
  // of dynamically creating the client
  if (dynamic_client_instance_ && sub_instance_) {
    dynamic_client_instance_ = false;
  }

  // Try again to create the client on first tick if this was not possible during construction or if client should be
  // created from a blackboard entry on the start of every iteration
  if (status() == BT::NodeStatus::IDLE && dynamic_client_instance_) {
    const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this);
    if (expected_name) {
      createSubscriber(expected_name.value());
    } else {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) +
        " - Cannot create the subscriber because the topic name couldn't be resolved using "
        "the communication port expression specified by the node's "
        "registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_PORT + ": " + context_.registration_options_.port +
        "). Error message: " + expected_name.error());
    }
  }

  if (!sub_instance_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - sub_instance_ is nullptr.");
  }

  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };
  auto status = check_status(onTick(last_msg_));
  last_msg_.reset();
  return status;
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::onTick(const std::shared_ptr<MessageT> & last_msg_ptr)
{
  if (!last_msg_ptr) return BT::NodeStatus::FAILURE;
  return onMessageReceived(*last_msg_ptr);
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::onMessageReceived(const MessageT & /*msg*/)
{
  return BT::NodeStatus::SUCCESS;
}

template <class MessageT>
inline std::string RosSubscriberNode<MessageT>::getTopicName() const
{
  if (sub_instance_) return sub_instance_->name;
  return "unkown";
}

template <class MessageT>
inline std::mutex & RosSubscriberNode<MessageT>::registryMutex()
{
  static std::mutex sub_mutex;
  return sub_mutex;
}

template <class MessageT>
inline typename RosSubscriberNode<MessageT>::SubscribersRegistry & RosSubscriberNode<MessageT>::getRegistry()
{
  static SubscribersRegistry subscribers_registry;
  return subscribers_registry;
}

}  // namespace auto_apms_behavior_tree::core