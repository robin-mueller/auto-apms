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
 * 2. Otherwise, use the value in RosNodeParams::default_port_name.
 */
template <class ServiceT>
class RosServiceNode : public BT::ActionNodeBase
{
public:
  using ServiceClient = typename rclcpp::Client<ServiceT>;
  using ServiceClientPtr = std::shared_ptr<ServiceClient>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   */
  explicit RosServiceNode(const std::string& instance_name, const BT::NodeConfig& conf, const RosNodeParams& params);

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

protected:
  struct ServiceClientInstance
  {
    ServiceClientInstance(std::shared_ptr<rclcpp::Node> node, const std::string& service_name);

    ServiceClientPtr service_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
  };

  static std::mutex& getMutex()
  {
    static std::mutex action_client_mutex;
    return action_client_mutex;
  }

  // method to set the service name programmatically
  void setServiceName(const std::string& service_name);

  rclcpp::Logger logger()
  {
    if (auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosServiceNode");
  }

  rclcpp::Time now()
  {
    if (auto node = node_.lock())
    {
      return node->now();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ServiceClientInstance>>;
  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry& getRegistry()
  {
    static ClientsRegistry clients_registry;
    return clients_registry;
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::string service_name_;
  bool service_name_should_be_checked_ = false;
  const std::chrono::milliseconds service_timeout_;
  const std::chrono::milliseconds wait_for_service_timeout_;

private:
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

template <class T>
inline RosServiceNode<T>::ServiceClientInstance::ServiceClientInstance(std::shared_ptr<rclcpp::Node> node,
                                                                       const std::string& service_name)
{
  callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor.add_callback_group(callback_group, node->get_node_base_interface());

  service_client = node->create_client<T>(service_name, rmw_qos_profile_services_default, callback_group);
}

template <class T>
inline RosServiceNode<T>::RosServiceNode(const std::string& instance_name, const BT::NodeConfig& conf,
                                         const RosNodeParams& params)
  : BT::ActionNodeBase(instance_name, conf)
  , node_(params.nh)
  , service_timeout_(params.request_timeout)
  , wait_for_service_timeout_(params.wait_for_server_timeout)
{
  // check port remapping
  auto portIt = config().input_ports.find("service_name");
  if (portIt != config().input_ports.end())
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
  if (!srv_instance_ && !params.default_port_name.empty())
  {
    createClient(params.default_port_name);
  }
}

template <class T>
inline bool RosServiceNode<T>::createClient(const std::string& service_name)
{
  if (service_name.empty() || service_name == "__default__placeholder__")
  {
    throw exceptions::RosNodeError("service_name is empty or invalid");
  }

  std::unique_lock lk(getMutex());
  auto node = node_.lock();
  if (!node)
  {
    throw exceptions::RosNodeError(
        "The ROS node went out of scope. RosNodeParams doesn't take the "
        "ownership of the node.");
  }
  auto client_key = std::string(node->get_fully_qualified_name()) + "/" + service_name;

  auto& registry = getRegistry();
  auto it = registry.find(client_key);
  if (it == registry.end() || it->second.expired())
  {
    srv_instance_ = std::make_shared<ServiceClientInstance>(node, service_name);
    registry.insert({ client_key, srv_instance_ });

    RCLCPP_INFO(logger(), "Node [%s] created service client [%s]", name().c_str(), service_name.c_str());
  }
  else
  {
    srv_instance_ = it->second.lock();
  }
  service_name_ = service_name;

  bool found = srv_instance_->service_client->wait_for_service(wait_for_service_timeout_);
  if (!found)
  {
    RCLCPP_ERROR(logger(), "%s: Service with name '%s' is not reachable.", name().c_str(), service_name_.c_str());
  }
  return found;
}

template <class T>
inline void RosServiceNode<T>::setServiceName(const std::string& service_name)
{
  service_name_ = service_name;
  createClient(service_name);
}

template <class T>
inline BT::NodeStatus RosServiceNode<T>::tick()
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
    throw exceptions::RosNodeError(
        "RosServiceNode: no service client was specified neither as "
        "default or in the ports");
  }

  auto CheckStatus = [](BT::NodeStatus status) {
    if (!isStatusCompleted(status))
    {
      throw exceptions::RosNodeError(
          "RosServiceNode: the callback must return either SUCCESS nor "
          "FAILURE");
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
      return CheckStatus(onFailure(INVALID_REQUEST));
    }

    // Check if server is ready
    if (!srv_instance_->service_client->service_is_ready())
    {
      return onFailure(SERVICE_UNREACHABLE);
    }

    future_response_ = srv_instance_->service_client->async_send_request(request).share();
    time_request_sent_ = now();

    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING)
  {
    srv_instance_->callback_executor.spin_some();

    // FIRST case: check if the goal request has a timeout
    if (!response_received_)
    {
      auto const nodelay = std::chrono::milliseconds(0);
      auto const timeout = rclcpp::Duration::from_seconds(double(service_timeout_.count()) / 1000);

      auto ret = srv_instance_->callback_executor.spin_until_future_complete(future_response_, nodelay);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        if ((now() - time_request_sent_) > timeout)
        {
          return CheckStatus(onFailure(SERVICE_TIMEOUT));
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
          throw exceptions::RosNodeError("Request was rejected by the service");
        }
      }
    }

    // SECOND case: response received
    return CheckStatus(onResponseReceived(response_));
  }
  return BT::NodeStatus::RUNNING;
}

template <class T>
inline void RosServiceNode<T>::halt()
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
  const std::string node_name =
      name() == registrationName() ? registrationName() : name() + "(" + registrationName() + ")";
  RCLCPP_ERROR(logger(), "%s - Error %d: %s", node_name.c_str(), error, toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace auto_apms_behavior_tree