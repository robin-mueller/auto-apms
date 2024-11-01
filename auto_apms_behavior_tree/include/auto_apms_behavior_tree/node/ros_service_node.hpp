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

#include "rclcpp/executors.hpp"
#include "behaviortree_cpp/action_node.h"

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/node/ros_node_context.hpp"

namespace auto_apms_behavior_tree
{

enum ServiceNodeErrorCode
{
  SERVICE_UNREACHABLE,
  SERVICE_TIMEOUT,
  INVALID_REQUEST,
  SERVICE_ABORTED
};

inline const char* toStr(const ServiceNodeErrorCode& err)
{
  switch (err)
  {
    case SERVICE_UNREACHABLE:
      return "SERVICE_UNREACHABLE";
    case SERVICE_TIMEOUT:
      return "SERVICE_TIMEOUT";
    case INVALID_REQUEST:
      return "INVALID_REQUEST";
    case SERVICE_ABORTED:
      return "SERVICE_ABORTED";
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
    ServiceClientInstance(std::shared_ptr<rclcpp::Node> node, const std::string& service_name);

    ServiceClientPtr service_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
  };

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ServiceClientInstance>>;

public:
  using ServiceType = ServiceT;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  explicit RosServiceNode(const std::string& instance_name, const Config& config, const Context& context);

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
    BT::PortsList basic = { BT::InputPort<std::string>("service_name", "", "Service name") };
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
  virtual bool setRequest(typename Request::SharedPtr& request);

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response);

  /** Callback invoked when something goes wrong; you can override it.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ServiceNodeErrorCode error);

  std::string getFullName() const;

protected:
  const Context& getRosContext();

  static std::mutex& getMutex();

  // method to set the service name programmatically
  void setServiceName(const std::string& service_name);

  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry& getRegistry();

  const rclcpp::Logger logger_;

private:
  const Context context_;
  std::string service_name_;
  bool service_name_should_be_checked_ = false;
  std::shared_ptr<ServiceClientInstance> srv_instance_;
  std::shared_future<typename Response::SharedPtr> future_response_;
  rclcpp::Time time_request_sent_;
  BT::NodeStatus on_feedback_state_change_;
  bool response_received_;
  typename Response::SharedPtr response_;

  bool createClient(const std::string& service_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class ServiceT>
inline RosServiceNode<ServiceT>::ServiceClientInstance::ServiceClientInstance(std::shared_ptr<rclcpp::Node> node,
                                                                              const std::string& service_name)
{
  callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor.add_callback_group(callback_group, node->get_node_base_interface());

  service_client = node->create_client<ServiceT>(service_name, rmw_qos_profile_services_default, callback_group);
}

template <class ServiceT>
inline RosServiceNode<ServiceT>::RosServiceNode(const std::string& instance_name, const Config& config,
                                                const Context& context)
  : BT::ActionNodeBase(instance_name, config), logger_(context.getLogger()), context_(context)
{
  // check port remapping
  auto portIt = config.input_ports.find("service_name");
  if (portIt != config.input_ports.end())
  {
    const std::string& bb_service_name = portIt->second;

    if (isBlackboardPointer(bb_service_name))
    {
      // unknown value at construction time. postpone to tick
      service_name_should_be_checked_ = true;
    }
    else if (!bb_service_name.empty())
    {
      // "hard-coded" name in the bb_service_name. Use it.
      createClient(bb_service_name);
    }
  }
  // no port value or it is empty. Use the default port value
  if (!srv_instance_ && !context.default_port_name.empty())
  {
    createClient(context.default_port_name);
  }
}

template <class ServiceT>
inline bool RosServiceNode<ServiceT>::createClient(const std::string& service_name)
{
  if (service_name.empty())
  {
    throw exceptions::RosNodeError(getFullName() + " - Argument service_name is empty when trying to create a client.");
  }

  std::unique_lock lk(getMutex());
  auto node = getRosContext().nh.lock();
  if (!node)
  {
    throw exceptions::RosNodeError(getFullName() +
                                   " - The shared pointer to the ROS node went out of scope. The tree node doesn't "
                                   "take the ownership of the node.");
  }
  auto client_key = std::string(node->get_fully_qualified_name()) + "/" + service_name;

  auto& registry = getRegistry();
  auto it = registry.find(client_key);
  if (it == registry.end() || it->second.expired())
  {
    srv_instance_ = std::make_shared<ServiceClientInstance>(node, service_name);
    registry.insert({ client_key, srv_instance_ });
    RCLCPP_DEBUG(logger_, "%s - Created client for service '%s'.", getFullName().c_str(), service_name.c_str());
  }
  else
  {
    srv_instance_ = it->second.lock();
  }
  service_name_ = service_name;

  bool found = srv_instance_->service_client->wait_for_service(getRosContext().wait_for_server_timeout);
  if (!found)
  {
    RCLCPP_WARN(logger_, "%s - Service with name '%s' is not reachable.", getFullName().c_str(), service_name_.c_str());
  }
  return found;
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::tick()
{
  if (!rclcpp::ok())
  {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // First, check if the service_client is valid and that the name of the
  // service_name in the port didn't change.
  // otherwise, create a new client
  if (!srv_instance_ || (status() == BT::NodeStatus::IDLE && service_name_should_be_checked_))
  {
    std::string service_name;
    getInput("service_name", service_name);
    if (service_name_ != service_name)
    {
      createClient(service_name);
    }
  }

  if (!srv_instance_)
  {
    throw exceptions::RosNodeError(getFullName() +
                                   " - You must specify a service name either by using a default value or by "
                                   "passing a value to the corresponding dynamic input port.");
  }

  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status))
    {
      throw exceptions::RosNodeError(getFullName() + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE)
  {
    setStatus(BT::NodeStatus::RUNNING);

    response_received_ = false;
    future_response_ = {};
    on_feedback_state_change_ = BT::NodeStatus::RUNNING;
    response_ = {};

    typename Request::SharedPtr request = std::make_shared<Request>();

    if (!setRequest(request))
    {
      return check_status(onFailure(INVALID_REQUEST));
    }

    // Check if server is ready
    if (!srv_instance_->service_client->service_is_ready())
    {
      return onFailure(SERVICE_UNREACHABLE);
    }

    future_response_ = srv_instance_->service_client->async_send_request(request).share();
    time_request_sent_ = getRosContext().getCurrentTime();

    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING)
  {
    srv_instance_->callback_executor.spin_some();

    // FIRST case: check if the goal request has a timeout
    if (!response_received_)
    {
      auto const nodelay = std::chrono::milliseconds(0);
      auto const timeout = rclcpp::Duration::from_seconds(double(getRosContext().request_timeout.count()) / 1000);

      auto ret = srv_instance_->callback_executor.spin_until_future_complete(future_response_, nodelay);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        if ((getRosContext().getCurrentTime() - time_request_sent_) > timeout)
        {
          return check_status(onFailure(SERVICE_TIMEOUT));
        }
        else
        {
          return BT::NodeStatus::RUNNING;
        }
      }
      else
      {
        response_received_ = true;
        response_ = future_response_.get();
        future_response_ = {};

        if (!response_)
        {
          throw exceptions::RosNodeError(getFullName() + " - Request was rejected by the service.");
        }
      }
    }

    // SECOND case: response received
    return check_status(onResponseReceived(response_));
  }
  return BT::NodeStatus::RUNNING;
}

template <class ServiceT>
inline void RosServiceNode<ServiceT>::halt()
{
  if (status() == BT::NodeStatus::RUNNING)
  {
    resetStatus();
  }
}

template <class ServiceT>
inline bool RosServiceNode<ServiceT>::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::onResponseReceived(const typename Response::SharedPtr& /*response*/)
{
  return BT::NodeStatus::SUCCESS;
}

template <class ServiceT>
inline BT::NodeStatus RosServiceNode<ServiceT>::onFailure(ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger_, "%s - Error %d: %s", getFullName().c_str(), error, toStr(error));
  return BT::NodeStatus::FAILURE;
}

template <class ServiceT>
inline std::string RosServiceNode<ServiceT>::getFullName() const
{
  // NOTE: registrationName() is empty during construction as this member is first set after the factory constructed the
  // object
  if (registrationName().empty())
    return name();
  if (this->name() == this->registrationName())
    return this->name();
  return this->name() + " (Type: " + this->registrationName() + ")";
}

template <class ServiceT>
inline const typename RosServiceNode<ServiceT>::Context& RosServiceNode<ServiceT>::getRosContext()
{
  return context_;
}

template <class ServiceT>
inline std::mutex& RosServiceNode<ServiceT>::getMutex()
{
  static std::mutex action_client_mutex;
  return action_client_mutex;
}

template <class ServiceT>
inline void RosServiceNode<ServiceT>::setServiceName(const std::string& service_name)
{
  service_name_ = service_name;
  createClient(service_name);
}

template <class ServiceT>
inline typename RosServiceNode<ServiceT>::ClientsRegistry& RosServiceNode<ServiceT>::getRegistry()
{
  static ClientsRegistry clients_registry;
  return clients_registry;
}

}  // namespace auto_apms_behavior_tree