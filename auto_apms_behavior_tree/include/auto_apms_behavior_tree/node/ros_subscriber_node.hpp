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
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/executors.hpp>
#include <string>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node/ros_params.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace auto_apms_behavior_tree
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
 * 2. Otherwise, use the value in RosNodeParams::default_port_name
 */
template <class TopicT>
class RosSubscriberNode : public BT::ConditionNode
{
public:
  using Subscriber = typename rclcpp::Subscription<TopicT>;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClass>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosSubscriberNode(const std::string& instance_name, const BT::NodeConfig& conf, const RosNodeParams& params,
                             const rclcpp::QoS& qos = { 10 });

  virtual ~RosSubscriberNode()
  {
    signal_connection_.disconnect();
  }

  /**
   * @brief Any subclass of RosTopicNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
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
   * @brief Callback invoked on every tick. You must return either SUCCESS or FAILURE.
   *
   * By default, this function calls RosSubscriberNode::onMessageReceived if a new message was received, otherwise it
   * immediately returns FAILURE.
   *
   * @param last_msg_ptr The latest message received, since the last tick. Will be `nullptr` if no new message was
   * received.
   * @return Status of the node, based on @p last_msg_ptr.
   */
  virtual BT::NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg_ptr);

  /**
   * @brief Callback invoked when a new message is received. You must return either SUCCESS or FAILURE.
   *
   * This callback won't be invoked if no new message has been received since the last time it was called. If you need
   * to evaluate an expression every time this node is ticked, refer to RosSubscriberNode::onTick instead.
   *
   * @param msg Most recently received message.
   * @return Status of the node based on @p msg.
   */
  virtual BT::NodeStatus onMessageReceived(const TopicT& msg);

protected:
  struct SubscriberInstance
  {
    SubscriberInstance(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, const rclcpp::QoS& qos);

    std::shared_ptr<Subscriber> subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    boost::signals2::signal<void(const std::shared_ptr<TopicT>)> broadcaster;
    std::shared_ptr<TopicT> last_msg;
  };

  static std::mutex& registryMutex()
  {
    static std::mutex sub_mutex;
    return sub_mutex;
  }

  using SubscribersRegistry = std::unordered_map<std::string, std::weak_ptr<SubscriberInstance>>;

  // contains the fully-qualified name of the node and the name of the topic
  static SubscribersRegistry& getRegistry()
  {
    static SubscribersRegistry subscribers_registry;
    return subscribers_registry;
  }

  rclcpp::Logger logger()
  {
    if (auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosSubscriberNode");
  }

  std::weak_ptr<rclcpp::Node> node_;
  const rclcpp::QoS qos_;
  std::shared_ptr<SubscriberInstance> sub_instance_;
  std::shared_ptr<TopicT> last_msg_;
  std::string topic_name_;
  boost::signals2::connection signal_connection_;
  std::string subscriber_key_;

private:
  bool createSubscriber(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosSubscriberNode<T>::RosSubscriberNode(const std::string& instance_name, const BT::NodeConfig& conf,
                                               const RosNodeParams& params, const rclcpp::QoS& qos)
  : BT::ConditionNode{ instance_name, conf }, node_{ params.nh }, qos_{ qos }
{
  // check port remapping
  auto portIt = config().input_ports.find("topic_name");
  if (portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if (bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if (params.default_port_name.empty())
      {
        throw std::logic_error(
            "Both [topic_name] in the BT::InputPort and the RosNodeParams "
            "are empty.");
      }
      else
      {
        createSubscriber(params.default_port_name);
      }
    }
    else if (!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createSubscriber(bb_topic_name);
    }
    else
    {
      // do nothing
      // createSubscriber will be invoked in the first tick().
    }
  }
  else
  {
    if (params.default_port_name.empty())
    {
      throw std::logic_error(
          "Both [topic_name] in the BT::InputPort and the RosNodeParams "
          "are empty.");
    }
    else
    {
      createSubscriber(params.default_port_name);
    }
  }
}

template <class T>
inline RosSubscriberNode<T>::SubscriberInstance::SubscriberInstance(std::shared_ptr<rclcpp::Node> node,
                                                                    const std::string& topic_name,
                                                                    const rclcpp::QoS& qos)
{
  // create a callback group for this particular instance
  callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor.add_callback_group(callback_group, node->get_node_base_interface());

  rclcpp::SubscriptionOptions option;
  option.callback_group = callback_group;

  // The callback will broadcast to all the instances of RosSubscriberNode<T>
  auto callback = [this](const std::shared_ptr<T> msg) {
    last_msg = msg;
    broadcaster(msg);
  };
  subscriber = node->create_subscription<T>(topic_name, qos, callback, option);
}

template <class T>
inline bool RosSubscriberNode<T>::createSubscriber(const std::string& topic_name)
{
  if (topic_name.empty())
  {
    throw exceptions::RosNodeError("topic_name is empty");
  }
  if (sub_instance_)
  {
    throw exceptions::RosNodeError("Can't call createSubscriber more than once");
  }

  // find SubscriberInstance in the registry
  std::unique_lock lk(registryMutex());

  auto node = node_.lock();
  if (!node)
  {
    throw exceptions::RosNodeError(
        "The ROS node went out of scope. RosNodeParams doesn't take the "
        "ownership of the node.");
  }
  subscriber_key_ = std::string(node->get_fully_qualified_name()) + "/" + topic_name;

  auto& registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if (it == registry.end() || it->second.expired())
  {
    sub_instance_ = std::make_shared<SubscriberInstance>(node, topic_name, qos_);
    registry.insert({ subscriber_key_, sub_instance_ });

    RCLCPP_INFO(logger(), "Node [%s] created Subscriber to topic [%s]", name().c_str(), topic_name.c_str());
  }
  else
  {
    sub_instance_ = it->second.lock();
  }

  // Check if there was a message received before the creation of this subscriber action
  if (sub_instance_->last_msg)
  {
    last_msg_ = sub_instance_->last_msg;
  }

  // add "this" as received of the broadcaster
  signal_connection_ = sub_instance_->broadcaster.connect([this](const std::shared_ptr<T> msg) { last_msg_ = msg; });

  topic_name_ = topic_name;
  return true;
}

template <class T>
inline BT::NodeStatus RosSubscriberNode<T>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  std::string topic_name;
  getInput("topic_name", topic_name);

  if (!topic_name.empty() && topic_name != "__default__placeholder__" && topic_name != topic_name_)
  {
    sub_instance_.reset();
  }

  if (!sub_instance_)
  {
    createSubscriber(topic_name);
  }

  auto CheckStatus = [](BT::NodeStatus status) {
    if (!isStatusCompleted(status))
    {
      throw std::logic_error(
          "RosSubscriberNode: the callback must return"
          "either SUCCESS or FAILURE");
    }
    return status;
  };
  sub_instance_->callback_group_executor.spin_some();
  auto status = CheckStatus(onTick(last_msg_));
  last_msg_.reset();
  return status;
}

template <class TopicT>
inline BT::NodeStatus RosSubscriberNode<TopicT>::onTick(const std::shared_ptr<TopicT>& last_msg_ptr)
{
  if (!last_msg_ptr)
    return BT::NodeStatus::FAILURE;
  return onMessageReceived(*last_msg_ptr);
}

template <class TopicT>
inline BT::NodeStatus RosSubscriberNode<TopicT>::onMessageReceived(const TopicT& /*msg*/)
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace auto_apms_behavior_tree