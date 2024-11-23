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
    SubscriberInstance(std::shared_ptr<rclcpp::Node> node, const std::string & topic_name, const rclcpp::QoS & qos);

    std::shared_ptr<Subscriber> subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    boost::signals2::signal<void(const std::shared_ptr<MessageT>)> broadcaster;
    std::shared_ptr<MessageT> last_msg;
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
    BT::PortsList basic = {BT::InputPort<std::string>("topic_name", "", "Topic name")};
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

  std::string getTopicName() const;

protected:
  const Context context_;

private:
  static std::mutex & registryMutex();

  // contains the fully-qualified name of the node and the name of the topic
  static SubscribersRegistry & getRegistry();

  bool createSubscriber(const std::string & topic_name);

  const rclcpp::Logger logger_;
  const rclcpp::QoS qos_;
  std::string topic_name_;
  bool topic_name_should_be_checked_ = false;
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
  std::shared_ptr<rclcpp::Node> node, const std::string & topic_name, const rclcpp::QoS & qos)
{
  // create a callback group for this particular instance
  callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor.add_callback_group(callback_group, node->get_node_base_interface());

  rclcpp::SubscriptionOptions option;
  option.callback_group = callback_group;

  // The callback will broadcast to all the instances of RosSubscriberNode<MessageT>
  auto callback = [this](const std::shared_ptr<MessageT> msg) {
    last_msg = msg;
    broadcaster(msg);
  };
  subscriber = node->create_subscription<MessageT>(topic_name, qos, callback, option);
}

template <class MessageT>
inline RosSubscriberNode<MessageT>::RosSubscriberNode(
  const std::string & instance_name, const Config & config, const Context & context, const rclcpp::QoS & qos)
: BT::ConditionNode{instance_name, config}, context_{context}, logger_(context.getLogger()), qos_{qos}
{
  // check port remapping
  auto portIt = config.input_ports.find("topic_name");
  if (portIt != config.input_ports.end()) {
    const std::string & bb_topic_name = portIt->second;

    if (isBlackboardPointer(bb_topic_name)) {
      // unknown value at construction time. postpone to tick
      topic_name_should_be_checked_ = true;
    } else if (!bb_topic_name.empty()) {
      // "hard-coded" name in the bb_topic_name. Use it.
      createSubscriber(bb_topic_name);
    }
  }
  // no port value or it is empty. Use the default port value
  if (!sub_instance_ && !context_.registration_params_.port.empty()) {
    createSubscriber(context_.registration_params_.port);
  }
}

template <class MessageT>
inline bool RosSubscriberNode<MessageT>::createSubscriber(const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) + " - Argument topic_name is empty when trying to create a client.");
  }
  if (sub_instance_) {
    throw exceptions::RosNodeError(context_.getFullName(this) + " - Cannot call createSubscriber() more than once.");
  }

  // find SubscriberInstance in the registry
  std::unique_lock lk(registryMutex());

  auto node = context_.nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) +
      " - The shared pointer to the ROS node went out of scope. The tree node doesn't "
      "take the ownership of the node.");
  }
  subscriber_key_ = std::string(node->get_fully_qualified_name()) + "/" + topic_name;

  auto & registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if (it == registry.end() || it->second.expired()) {
    sub_instance_ = std::make_shared<SubscriberInstance>(node, topic_name, qos_);
    registry.insert({subscriber_key_, sub_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created subscriber for topic '%s'.", context_.getFullName(this).c_str(), topic_name.c_str());
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

  topic_name_ = topic_name;
  return true;
}

template <class MessageT>
inline BT::NodeStatus RosSubscriberNode<MessageT>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if (!sub_instance_ || (status() == BT::NodeStatus::IDLE && topic_name_should_be_checked_)) {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if (topic_name_ != topic_name) {
      createSubscriber(topic_name);
    }
  }

  if (!sub_instance_) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) +
      " - You must specify a service name either by using a default value or by "
      "passing a value to the corresponding dynamic input port.");
  }

  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };
  sub_instance_->callback_group_executor.spin_some();
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
  return topic_name_;
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