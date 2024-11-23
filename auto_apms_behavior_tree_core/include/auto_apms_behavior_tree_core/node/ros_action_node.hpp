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
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_tree::core
{

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  INVALID_GOAL
};

inline const char * toStr(const ActionNodeErrorCode & err)
{
  switch (err) {
    case SERVER_UNREACHABLE:
      return "SERVER_UNREACHABLE";
    case SEND_GOAL_TIMEOUT:
      return "SEND_GOAL_TIMEOUT";
    case GOAL_REJECTED_BY_SERVER:
      return "GOAL_REJECTED_BY_SERVER";
    case INVALID_GOAL:
      return "INVALID_GOAL";
  }
  return nullptr;
}

/**
 * @brief Abstract class to wrap rclcpp_action::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
 *
 * the corresponding wrapper would be:
 *
 * class FibonacciNode: public RosActionNode<action_tutorials_interfaces::action::Fibonacci>
 *
 * RosActionNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the action will be determined as follows:
 *
 * 1. If a value is passes in the BT::InputPort "action_name", use that
 * 2. Otherwise, use the value in RosNodeContext::default_port_name.
 */
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;

  struct ActionClientInstance
  {
    ActionClientInstance(std::shared_ptr<rclcpp::Node> node, const std::string & action_name);

    ActionClientPtr action_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
    typename ActionClient::SendGoalOptions goal_options;
  };

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ActionClientInstance>>;

public:
  using ActionType = ActionT;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

  inline static const std::string INPUT_KEY_ACTION_PORT = "action_name";

  explicit RosActionNode(const std::string & instance_name, const Config & config, const Context & context);

  virtual ~RosActionNode() = default;

  /**
   * @brief Any subclass of RosActionNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {BT::InputPort<std::string>(INPUT_KEY_ACTION_PORT, "", "Action server name")};
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /// @brief  Callback executed when the node is halted. Note that cancelGoal()
  /// is done automatically.
  virtual void onHalt();

  /** setGoal s a callback that allows the user to set
   *  the goal message (ActionT::Goal).
   *
   * @param goal  the goal to be sent to the action server.
   *
   * @return false if the request should not be sent. In that case,
   * RosActionNode::onFailure(INVALID_GOAL) will be called.
   */
  virtual bool setGoal(Goal & goal);

  /** Callback invoked when the result is received by the server.
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResultReceived(const WrappedResult & result);

  /** Callback invoked when the feedback is received.
   * It generally returns RUNNING, but the user can also use this callback to cancel the
   * current action and return SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback_ptr);

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error);

  /// Method used to send a request to the Action server to cancel the current goal
  void cancelGoal();

  /// The default halt() implementation will call cancelGoal if necessary.
  void halt() override;

  BT::NodeStatus tick() override;

  /// Can be used to change the name of the action programmatically
  void setActionName(const std::string & action_name);

  std::string getActionName() const;

protected:
  const Context context_;

private:
  static std::mutex & getMutex();

  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry & getRegistry();

  bool createClient(const std::string & action_name);

  const rclcpp::Logger logger_;
  std::string action_name_;
  std::shared_ptr<ActionClientInstance> client_instance_;
  bool action_name_should_be_checked_ = false;
  std::string action_client_key_;
  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time time_goal_sent_;
  BT::NodeStatus on_feedback_state_change_;
  bool goal_response_;
  WrappedResult result_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ActionT>
RosActionNode<ActionT>::ActionClientInstance::ActionClientInstance(
  std::shared_ptr<rclcpp::Node> node, const std::string & action_name)
{
  callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_executor.add_callback_group(callback_group, node->get_node_base_interface());
  action_client = rclcpp_action::create_client<ActionT>(node, action_name, callback_group);
}

template <class ActionT>
inline RosActionNode<ActionT>::RosActionNode(
  const std::string & instance_name, const Config & config, const Context & context)
: BT::ActionNodeBase(instance_name, config), context_(context), logger_(context.getLogger())
{
  // Three cases:
  // - we use the default action_name in RosNodeContext when port is empty
  // - we use the action_name in the port and it is a static string.
  // - we use the action_name in the port and it is blackboard entry.

  // check port remapping
  auto portIt = config.input_ports.find(INPUT_KEY_ACTION_PORT);
  if (portIt != config.input_ports.end()) {
    const std::string & bb_service_name = portIt->second;

    if (isBlackboardPointer(bb_service_name)) {
      // unknown value at construction time. Postpone to tick
      action_name_should_be_checked_ = true;
    } else if (!bb_service_name.empty()) {
      // "hard-coded" name in the bb_service_name. Use it.
      createClient(bb_service_name);
    }
  }
  // no port value or it is empty. Use the default value
  if (!client_instance_ && !context_.registration_params_.port.empty()) {
    createClient(context_.registration_params_.port);
  }
}

template <class ActionT>
inline bool RosActionNode<ActionT>::createClient(const std::string & action_name)
{
  if (action_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) + " - Argument action_name is empty when trying to create a client.");
  }

  std::unique_lock lk(getMutex());
  auto node = context_.nh_.lock();
  if (!node) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) +
      " - The shared pointer to the ROS node went out of scope. The tree node doesn't "
      "take the ownership of the node.");
  }
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;

  auto & registry = getRegistry();
  auto it = registry.find(action_client_key_);
  if (it == registry.end() || it->second.expired()) {
    client_instance_ = std::make_shared<ActionClientInstance>(node, action_name);
    registry.insert({action_client_key_, client_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created client for action '%s'.", context_.getFullName(this).c_str(), action_name.c_str());
  } else {
    client_instance_ = it->second.lock();
  }

  action_name_ = action_name;

  bool found = client_instance_->action_client->wait_for_action_server(context_.registration_params_.wait_timeout);
  if (!found) {
    std::string msg =
      context_.getFullName(this) + " - Action server with name '" + action_name_ + "' is not reachable.";
    if (context_.registration_params_.allow_unreachable) {
      RCLCPP_WARN_STREAM(logger_, msg);
    } else {
      RCLCPP_ERROR_STREAM(logger_, msg);
      throw exceptions::RosNodeError(msg);
    }
  }
  return found;
}

template <class ActionT>
inline void RosActionNode<ActionT>::onHalt()
{
}

template <class ActionT>
inline bool RosActionNode<ActionT>::setGoal(Goal & /*goal*/)
{
  return true;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onResultReceived(const WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onFeedback(const std::shared_ptr<const Feedback> /*feedback_ptr*/)
{
  return BT::NodeStatus::RUNNING;
}

template <class ActionT>
inline BT::NodeStatus RosActionNode<ActionT>::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger_, "%s - Error %d: %s", context_.getFullName(this).c_str(), error, toStr(error));
  return BT::NodeStatus::FAILURE;
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  auto & executor = client_instance_->callback_executor;
  if (!goal_response_ && future_goal_handle_.valid()) {
    // Here the discussion is if we should block or put a timer for the waiting
    auto ret = executor.spin_until_future_complete(future_goal_handle_, context_.registration_params_.request_timeout);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      // Do nothing in case of INTERRUPT or TIMEOUT since we must return rather quickly
      return;
    }
    goal_handle_ = future_goal_handle_.get();
    future_goal_handle_ = {};
  }

  // If goal was rejected or handle has been invalidated, we do not need to cancel
  if (!goal_handle_) return;

  /**
   * Wait for the cancellation to be complete
   */

  auto future_cancel = client_instance_->action_client->async_cancel_goal(goal_handle_);
  if (const auto code =
        executor.spin_until_future_complete(future_cancel, context_.registration_params_.request_timeout);
      code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "%s - Failed to wait until cancellation of action '%s' is complete (Received: %s). Ignoring.",
      context_.getFullName(this).c_str(), action_name_.c_str(), rclcpp::to_string(code).c_str());
  }
}

template <class T>
inline void RosActionNode<T>::halt()
{
  if (status() == BT::NodeStatus::RUNNING) {
    cancelGoal();
    onHalt();
  }
}

template <class T>
inline BT::NodeStatus RosActionNode<T>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    return BT::NodeStatus::FAILURE;
  }

  // First, check if the action_client_ is valid and that the name of the
  // action_name in the port didn't change.
  // otherwise, create a new client
  if (!client_instance_ || (status() == BT::NodeStatus::IDLE && action_name_should_be_checked_)) {
    std::string action_name;
    getInput(INPUT_KEY_ACTION_PORT, action_name);
    if (action_name_ != action_name) {
      createClient(action_name);
    }
  }

  if (!client_instance_) {
    throw exceptions::RosNodeError(
      context_.getFullName(this) +
      " - You must specify an action name either by using a default value or by "
      "passing a value to the corresponding dynamic input port.");
  }

  auto & action_client = client_instance_->action_client;

  //------------------------------------------
  auto check_status = [this](BT::NodeStatus status) {
    if (!isStatusCompleted(status)) {
      throw exceptions::RosNodeError(
        context_.getFullName(this) + " - The callback must return either SUCCESS or FAILURE.");
    }
    return status;
  };

  // first step to be done only at the beginning of the Action
  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);

    goal_response_ = false;
    future_goal_handle_ = {};
    on_feedback_state_change_ = BT::NodeStatus::RUNNING;
    result_ = {};

    Goal goal;
    if (!setGoal(goal)) {
      return check_status(onFailure(INVALID_GOAL));
    }

    typename ActionClient::SendGoalOptions goal_options;

    //--------------------
    goal_options.feedback_callback = [this](
                                       typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback) {
      on_feedback_state_change_ = onFeedback(feedback);
      if (on_feedback_state_change_ == BT::NodeStatus::IDLE) {
        throw std::logic_error(context_.getFullName(this) + " - onFeedback() must not return IDLE.");
      }
      emitWakeUpSignal();
    };
    //--------------------
    goal_options.result_callback = [this](const WrappedResult & result) {
      if (!goal_handle_)
        throw std::logic_error(context_.getFullName(this) + " - goal_handle_ is nullptr in result callback.");
      if (goal_handle_->get_goal_id() == result.goal_id) {
        result_ = result;
        goal_handle_ = nullptr;  // Reset internal goal handle
        emitWakeUpSignal();
      }
    };
    //--------------------
    // Check if server is ready
    if (!action_client->action_server_is_ready()) {
      return onFailure(SERVER_UNREACHABLE);
    }

    future_goal_handle_ = action_client->async_send_goal(goal, goal_options);
    time_goal_sent_ = context_.getCurrentTime();

    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING) {
    std::unique_lock<std::mutex> lock(getMutex());
    client_instance_->callback_executor.spin_some();

    // FIRST case: check if the goal request has a timeout
    if (!goal_response_) {
      auto ret =
        client_instance_->callback_executor.spin_until_future_complete(future_goal_handle_, std::chrono::seconds(0));
      if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        if ((context_.getCurrentTime() - time_goal_sent_) > context_.registration_params_.request_timeout) {
          return check_status(onFailure(SEND_GOAL_TIMEOUT));
        }
        return BT::NodeStatus::RUNNING;
      } else {
        goal_response_ = true;
        goal_handle_ = future_goal_handle_.get();
        future_goal_handle_ = {};

        if (!goal_handle_) {
          RCLCPP_ERROR(logger_, "%s - Goal was rejected by server.", context_.getFullName(this).c_str());
          return check_status(onFailure(GOAL_REJECTED_BY_SERVER));
        }
        RCLCPP_DEBUG(logger_, "%s - Goal accepted by server, waiting for result.", context_.getFullName(this).c_str());
      }
    }

    // SECOND case: onFeedback requested a stop
    if (on_feedback_state_change_ != BT::NodeStatus::RUNNING) {
      cancelGoal();
      return on_feedback_state_change_;
    }

    // THIRD case: result received
    if (result_.code != rclcpp_action::ResultCode::UNKNOWN) return check_status(onResultReceived(result_));
  }
  return BT::NodeStatus::RUNNING;
}

template <class T>
inline void RosActionNode<T>::setActionName(const std::string & action_name)
{
  action_name_ = action_name;
  createClient(action_name);
}

template <class ActionT>
inline std::string RosActionNode<ActionT>::getActionName() const
{
  return action_name_;
}

template <class ActionT>
inline std::mutex & RosActionNode<ActionT>::getMutex()
{
  static std::mutex action_client_mutex;
  return action_client_mutex;
}

template <class ActionT>
inline typename RosActionNode<ActionT>::ClientsRegistry & RosActionNode<ActionT>::getRegistry()
{
  static ClientsRegistry action_clients_registry;
  return action_clients_registry;
}

}  // namespace auto_apms_behavior_tree::core