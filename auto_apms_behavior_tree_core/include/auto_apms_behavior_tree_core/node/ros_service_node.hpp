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
#include <memory>
#include <string>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/ros_node_context.hpp"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/executors.hpp"

namespace auto_apms_behavior_tree::core
{

enum ServiceNodeErrorCode
{
  SERVICE_UNREACHABLE,
  SERVICE_TIMEOUT,
  INVALID_REQUEST
};

inline const char * toStr(const ServiceNodeErrorCode & err)
{
  switch (err) {
    case SERVICE_UNREACHABLE:
      return "SERVICE_UNREACHABLE";
    case SERVICE_TIMEOUT:
      return "SERVICE_TIMEOUT";
    case INVALID_REQUEST:
      return "INVALID_REQUEST";
  }
  return nullptr;
}

/**
 * @brief Abstract class use to wrap rclcpp::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
 *
 * the corresponding wrapper would be:
 *
 * class AddTwoNumbers: public RosServiceNode<example_interfaces::srv::AddTwoInts>
 *
 * RosServiceNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the service will be determined as follows:
 *
 * 1. If a value is passes in the BT::InputPort "service_name", use that
 * 2. Otherwise, use the value in RosNodeContext::default_port_name.
 */
template <class ServiceT>
class RosServiceNode : public BT::ActionNodeBase
{
  using ServiceClient = typename rclcpp::Client<ServiceT>;
  using ServiceClientPtr = std::shared_ptr<ServiceClient>;

  struct ServiceClientInstance
  {
    ServiceClientInstance(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & service_name);

    ServiceClientPtr service_client;
    std::string name;
  };

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ServiceClientInstance>>;

public:
  using ServiceType = ServiceT;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  explicit RosServiceNode(const std::string & instance_name, const Config & config, const Context & context);

  virtual ~RosServiceNode() = default;

  /**
   * @brief Any subclass of RosServiceNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>("port", "Name of the ROS 2 service.")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  BT::NodeStatus tick() override;

  /// The default halt() implementation.
  void halt() override;

  /** setRequest is a callback that allows the user to set
   * the request message (ServiceT::Request).
   *
   * @param request  the request to be sent to the service provider.
   *
   * @return false if the request should not be sent. In that case,
   * RosServiceNode::onFailure(INVALID_REQUEST) will be called.
   */
  virtual bool setRequest(typename Request::SharedPtr & request);

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr & response);

  /** Callback invoked when something goes wrong; you can override it.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error);

  bool createClient(const std::string & service_name);

  std::string getServiceName() const;

protected:
  const Context context_;

private:
  static std::mutex & getMutex();

  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry & getRegistry();

private:
  const rclcpp::Logger logger_;
  bool dynamic_client_instance_ = false;
  std::shared_ptr<ServiceClientInstance> client_instance_;
  typename ServiceClient::SharedFuture future_;
  int64_t request_id_;
  rclcpp::Time time_request_sent_;
  BT::NodeStatus on_feedback_state_change_;
  typename Response::SharedPtr response_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ServiceT>
inline RosServiceNode<ServiceT>::ServiceClientInstance::ServiceClientInstance(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & service_name)
{
  service_client = node->create_client<ServiceT>(service_name, rmw_qos_profile_services_default, group);
  name = service_name;
}

template <class ServiceT>
inline RosServiceNode<ServiceT>::RosServiceNode(
  const std::string & instance_name, const Config & config, const Context & context)
: BT::ActionNodeBase(instance_name, config), context_(context), logger_(context.getLogger())
{
  if (const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this)) {
    createClient(expected_name.value());
  } else {
    // We assume that determining the communication port requires a blackboard pointer, which cannot be evaluated at
    // construction time. The expression will be evaluated each time before the node is ticked the first time after
    // successful execution.
    dynamic_client_instance_ = true;
  }
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // If client has been set up in derived constructor, event though this constructor couldn't, we discard the intention
  // of dynamically creating the client
  if (dynamic_client_instance_ && client_instance_) {
    dynamic_client_instance_ = false;
  }

  // Try again to create the client on first tick if this was not possible during construction or if client should be
  // created from a blackboard entry on the start of every iteration
  if (status() == BT::NodeStatus::IDLE && dynamic_client_instance_) {
    const BT::Expected<std::string> expected_name = context_.getCommunicationPortName(this);
    if (expected_name) {
      createClient(expected_name.value());
    } else {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) +
        " - Cannot create the service client because the service name couldn't be resolved using "
        "the communication port expression specified by the node's "
        "registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_PORT + ": " + context_.registration_options_.port +
        "). Error message: " + expected_name.error());
    }
  }

  if (!client_instance_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - client_instance_ is nullptr.");
  }

  auto & service_client = client_instance_->service_client;

  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);

    on_feedback_state_change_ = BT::NodeStatus::RUNNING;
    response_ = {};

    typename Request::SharedPtr request = std::make_shared<Request>();

    if (!setRequest(request)) {
      return check_status(onFailure(INVALID_REQUEST));
    }

    // Check if server is ready
    if (!service_client->service_is_ready()) {
      return onFailure(SERVICE_UNREACHABLE);
    }

    const auto future_and_request_id =
      service_client->async_send_request(request, [this](typename ServiceClient::SharedFuture response) {
        if (response.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          throw exceptions::RosNodeError(
            this->context_.getFullyQualifiedTreeNodeName(this) + " - Response not ready in response callback.");
        }
        this->response_ = response.get();
      });
    future_ = future_and_request_id.future;
    request_id_ = future_and_request_id.request_id;
    time_request_sent_ = context_.getCurrentTime();

    RCLCPP_DEBUG(logger_, "%s - Service request sent.", context_.getFullyQualifiedTreeNodeName(this).c_str());
    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING) {
    // FIRST case: check if the goal request has a timeout
    if (!response_) {
      // See if we must time out
      if ((context_.getCurrentTime() - time_request_sent_) > context_.registration_options_.request_timeout) {
        // Remove the pending request with the client if timed out
        client_instance_->service_client->remove_pending_request(request_id_);
        return check_status(onFailure(SERVICE_TIMEOUT));
      }
      return BT::NodeStatus::RUNNING;
    } else if (future_.valid()) {
      // Invalidate future since it's obsolete now and it indicates that we've done this step
      future_ = {};

      RCLCPP_DEBUG(logger_, "%s - Service response received.", context_.getFullyQualifiedTreeNodeName(this).c_str());
    }

    // SECOND case: response received
    return check_status(onResponseReceived(response_));
  }
  return BT::NodeStatus::RUNNING;
}

template <class ServiceT>
inline void RosServiceNode<ServiceT>::halt()
{
  if (status() == BT::NodeStatus::RUNNING) {
    resetStatus();
  }
}

template <class ServiceT>
inline bool RosServiceNode<ServiceT>::setRequest(typename Request::SharedPtr & /*request*/)
{
  return true;
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::onResponseReceived(const typename Response::SharedPtr & /*response*/)
{
  return BT::NodeStatus::SUCCESS;
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::onFailure(ServiceNodeErrorCode error)
{
  const std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Unexpected error " +
                          std::to_string(error) + ": " + toStr(error) + ".";
  RCLCPP_ERROR_STREAM(logger_, msg);
  throw exceptions::RosNodeError(msg);
}

template <class ServiceT>
inline bool RosServiceNode<ServiceT>::createClient(const std::string & service_name)
{
  if (service_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - Argument service_name is empty when trying to create the client.");
  }

  // Check if the service with given name is already set up
  if (
    client_instance_ && service_name == client_instance_->name &&
    client_instance_->service_client->service_is_ready()) {
    return true;
  }

  std::unique_lock lk(getMutex());

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
  auto client_key = std::string(node->get_fully_qualified_name()) + "/" + service_name;

  auto & registry = getRegistry();
  auto it = registry.find(client_key);
  if (it == registry.end() || it->second.expired()) {
    client_instance_ = std::make_shared<ServiceClientInstance>(node, group, service_name);
    registry.insert({client_key, client_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created client for service '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      service_name.c_str());
  } else {
    client_instance_ = it->second.lock();
  }

  bool found = client_instance_->service_client->wait_for_service(context_.registration_options_.wait_timeout);
  if (!found) {
    std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Service with name '" + client_instance_->name +
                      "' is not reachable.";
    if (context_.registration_options_.allow_unreachable) {
      RCLCPP_WARN_STREAM(logger_, msg);
    } else {
      RCLCPP_ERROR_STREAM(logger_, msg);
      throw exceptions::RosNodeError(msg);
    }
  }
  return found;
}

template <class ServiceT>
inline std::string RosServiceNode<ServiceT>::getServiceName() const
{
  if (client_instance_) return client_instance_->name;
  return "unkown";
}

template <class ServiceT>
inline std::mutex & RosServiceNode<ServiceT>::getMutex()
{
  static std::mutex action_client_mutex;
  return action_client_mutex;
}

template <class ServiceT>
inline typename RosServiceNode<ServiceT>::ClientsRegistry & RosServiceNode<ServiceT>::getRegistry()
{
  static ClientsRegistry clients_registry;
  return clients_registry;
}

}  // namespace auto_apms_behavior_tree::core