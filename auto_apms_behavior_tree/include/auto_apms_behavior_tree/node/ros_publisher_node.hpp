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
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/executors.hpp>
#include <string>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node/ros_params.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace auto_apms_behavior_tree
{

/**
 * @brief Abstract class to wrap a ROS publisher
 *
 */
template <class TopicT>
class RosPublisherNode : public BT::ConditionNode
{
public:
  using Publisher = typename rclcpp::Publisher<TopicT>;

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClass>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosPublisherNode(const std::string& instance_name, const BT::NodeConfig& conf, const RosNodeParams& params,
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
  virtual bool setMessage(TopicT& msg);

protected:
  std::shared_ptr<rclcpp::Node> node_;
  const rclcpp::QoS qos_;
  std::string prev_topic_name_;
  bool topic_name_may_change_ = false;

private:
  std::shared_ptr<Publisher> publisher_;

  bool createPublisher(const std::string& topic_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosPublisherNode<T>::RosPublisherNode(const std::string& instance_name, const BT::NodeConfig& conf,
                                             const RosNodeParams& params, const rclcpp::QoS& qos)
  : BT::ConditionNode(instance_name, conf), node_(params.nh), qos_{ qos }
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
        createPublisher(params.default_port_name);
      }
    }
    else if (!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createPublisher(bb_topic_name);
    }
    else
    {
      topic_name_may_change_ = true;
      // createPublisher will be invoked in the first tick().
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
      createPublisher(params.default_port_name);
    }
  }
}

template <class T>
inline bool RosPublisherNode<T>::createPublisher(const std::string& topic_name)
{
  if (topic_name.empty())
  {
    throw exceptions::RosNodeError("topic_name is empty");
  }

  publisher_ = node_->create_publisher<T>(topic_name, qos_);
  prev_topic_name_ = topic_name;
  return true;
}

template <class T>
inline BT::NodeStatus RosPublisherNode<T>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if (!publisher_ || (status() == BT::NodeStatus::IDLE && topic_name_may_change_))
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    if (prev_topic_name_ != topic_name)
    {
      createPublisher(topic_name);
    }
  }

  T msg;
  if (!setMessage(msg))
  {
    return BT::NodeStatus::FAILURE;
  }
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

template <class TopicT>
inline bool RosPublisherNode<TopicT>::setMessage(TopicT& /*msg*/)
{
  return true;
}

}  // namespace auto_apms_behavior_tree