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

#include "rclcpp/qos.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node/ros_node_context.hpp"

namespace auto_apms_behavior_tree
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

  explicit RosPublisherNode(const std::string& instance_name, const Config& config, const Context& context,
                            const rclcpp::QoS& qos = { 10 });

  virtual ~RosPublisherNode() = default;

  /**
   * @brief Any subclass of RosPublisherNode that has additinal ports must provide a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = { BT::InputPort<std::string>("topic_name", "", "Topic name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  BT::NodeStatus tick() override final;

  /**
   * @brief setMessage is a callback invoked in tick to allow the user to pass
   * the message to be published.
   *
   * @param msg the message.
   * @return  return false if anything is wrong and we must not send the message.
   * the Condition will return FAILURE.
   */
  virtual bool setMessage(MessageT& msg);

  std::string getFullName() const;

protected:
  const Context& getRosContext();

  const rclcpp::Logger logger_;

private:
  const Context context_;
  const rclcpp::QoS qos_;
  std::string topic_name_;
  bool topic_name_should_be_checked_ = false;
  std::shared_ptr<Publisher> publisher_;

  bool createPublisher(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class MessageT>
inline RosPublisherNode<MessageT>::RosPublisherNode(const std::string& instance_name, const Config& config,
                                                    const Context& context, const rclcpp::QoS& qos)
  : BT::ConditionNode(instance_name, config), logger_(context.getLogger()), context_(context), qos_{ qos }
{
  // check port remapping
  auto portIt = config.input_ports.find("topic_name");
  if (portIt != config.input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if (isBlackboardPointer(bb_topic_name))
    {
      // unknown value at construction time. postpone to tick
      topic_name_should_be_checked_ = true;
    }
    else if (!bb_topic_name.empty())
    {
      // "hard-coded" name in the bb_topic_name. Use it.
      createPublisher(bb_topic_name);
    }
  }
  // no port value or it is empty. Use the default port value
  if (!publisher_ && !context.default_port_name.empty())
  {
    createPublisher(context.default_port_name);
  }
}

template <class MessageT>
inline bool RosPublisherNode<MessageT>::createPublisher(const std::string& topic_name)
{
  if (topic_name.empty())
  {
    throw exceptions::RosNodeError(getFullName() + " - Argument topic_name is empty when trying to create a client.");
  }

  auto node = getRosContext().nh.lock();
  if (!node)
  {
    throw exceptions::RosNodeError(getFullName() +
                                   " - The shared pointer to the ROS node went out of scope. The tree node doesn't "
                                   "take the ownership of the node.");
  }

  publisher_ = node->template create_publisher<MessageT>(topic_name, qos_);
  topic_name_ = topic_name;
  RCLCPP_DEBUG(logger_, "%s - Created publisher for topic '%s'.", getFullName().c_str(), topic_name.c_str());
  return true;
}

template <class MessageT>
inline BT::NodeStatus RosPublisherNode<MessageT>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if (!publisher_ || (status() == BT::NodeStatus::IDLE && topic_name_should_be_checked_))
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if (topic_name_ != topic_name)
    {
      createPublisher(topic_name);
    }
  }

  if (!publisher_)
  {
    throw exceptions::RosNodeError(getFullName() +
                                   " - You must specify a service name either by using a default value or by "
                                   "passing a value to the corresponding dynamic input port.");
  }

  MessageT msg;
  if (!setMessage(msg))
  {
    return BT::NodeStatus::FAILURE;
  }
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

template <class MessageT>
inline bool RosPublisherNode<MessageT>::setMessage(MessageT& /*msg*/)
{
  return true;
}

template <class MessageT>
inline std::string RosPublisherNode<MessageT>::getFullName() const
{
  // NOTE: registrationName() is empty during construction as this member is frist set after the factory constructed the
  // object
  if (registrationName().empty())
    return name();
  if (this->name() == this->registrationName())
    return this->name();
  return this->name() + " (Type: " + this->registrationName() + ")";
}

template <class MessageT>
inline const typename RosPublisherNode<MessageT>::Context& RosPublisherNode<MessageT>::getRosContext()
{
  return context_;
}

}  // namespace auto_apms_behavior_tree