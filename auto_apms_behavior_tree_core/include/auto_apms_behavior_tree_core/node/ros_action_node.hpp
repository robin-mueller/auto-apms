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
    ActionClientInstance(
      rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & action_name);

    ActionClientPtr action_client;
    std::string name;
  };

  using ClientsRegistry = std::unordered_map<std::string, std::weak_ptr<ActionClientInstance>>;

public:
  using ActionType = ActionT;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Config = BT::NodeConfig;
  using Context = RosNodeContext;

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
    BT::PortsList basic = {BT::InputPort<std::string>("port", "Name of the ROS 2 action.")};
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

  bool createClient(const std::string & action_name);

  std::string getActionName() const;

protected:
  const Context context_;

private:
  static std::mutex & getMutex();

  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry & getRegistry();

  const rclcpp::Logger logger_;
  bool dynamic_client_instance_ = false;
  std::shared_ptr<ActionClientInstance> client_instance_;
  std::string action_client_key_;
  std::shared_future<typename GoalHandle::SharedPtr> future_goal_handle_;
  typename GoalHandle::SharedPtr goal_handle_;
  rclcpp::Time time_goal_sent_;
  BT::NodeStatus on_feedback_state_change_;
  bool goal_response_received_;  // We must use this additional flag because goal_handle_ may be nullptr if rejected
  WrappedResult result_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ActionT>
RosActionNode<ActionT>::ActionClientInstance::ActionClientInstance(
  rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr group, const std::string & action_name)
{
  action_client = rclcpp_action::create_client<ActionT>(node, action_name, group);
  name = action_name;
}

template <class ActionT>
inline RosActionNode<ActionT>::RosActionNode(
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
  std::string result_str;
  switch (result.code) {
    case rclcpp_action::ResultCode::ABORTED:
      result_str = "ABORTED";
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_str = "CANCELED";
      break;
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_str = "SUCCEEDED";
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      result_str = "UNKNOWN";
      break;
  }
  RCLCPP_DEBUG(
    logger_, "%s - Goal completed. Received result %s.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
    result_str.c_str());
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
  const std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Unexpected error " +
                          std::to_string(error) + ": " + toStr(error) + ".";
  RCLCPP_ERROR_STREAM(logger_, msg);
  throw exceptions::RosNodeError(msg);
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_ptr = context_.executor_.lock();
  if (!executor_ptr) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - Cannot cancel goal for action '" + client_instance_->name +
      "' since the pointer to the associated ROS 2 executor expired.");
  }

  if (future_goal_handle_.valid()) {
    // Here the discussion is if we should block or put a timer for the waiting
    const rclcpp::FutureReturnCode ret =
      executor_ptr->spin_until_future_complete(future_goal_handle_, context_.registration_options_.request_timeout);
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

  std::shared_future<std::shared_ptr<typename ActionClient::CancelResponse>> future_cancel =
    client_instance_->action_client->async_cancel_goal(goal_handle_);
  if (const auto code =
        executor_ptr->spin_until_future_complete(future_cancel, context_.registration_options_.request_timeout);
      code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(
      logger_, "%s - Failed to wait until goal for action '%s' was cancelled successfully (Received: %s).",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), client_instance_->name.c_str(),
      rclcpp::to_string(code).c_str());
  }
}

template <class T>
inline void RosActionNode<T>::halt()
{
  if (status() == BT::NodeStatus::RUNNING) {
    cancelGoal();
    onHalt();
    resetStatus();
  }
}

template <class T>
inline BT::NodeStatus RosActionNode<T>::tick()
{
  if (!rclcpp::ok()) {
    halt();
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) + " - ROS 2 context has been shut down.");
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
        " - Cannot create the action client because the action name couldn't be resolved using "
        "the communication port expression specified by the node's "
        "registration parameters (" +
        NodeRegistrationOptions::PARAM_NAME_PORT + ": " + context_.registration_options_.port +
        "). Error message: " + expected_name.error());
    }
  }

  if (!client_instance_) {
    throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - client_instance_ is nullptr.");
  }

  auto & action_client = client_instance_->action_client;

  //------------------------------------------
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

    goal_response_received_ = false;
    on_feedback_state_change_ = BT::NodeStatus::RUNNING;
    result_ = {};

    // Check if server is ready
    if (!action_client->action_server_is_ready()) {
      return onFailure(SERVER_UNREACHABLE);
    }

    Goal goal;
    if (!setGoal(goal)) {
      return check_status(onFailure(INVALID_GOAL));
    }

    typename ActionClient::SendGoalOptions goal_options;
    goal_options.goal_response_callback = [this](typename GoalHandle::SharedPtr goal_handle) {
      // Indicate that a goal response has been received and let tick() do the rest
      this->goal_response_received_ = true;
      this->goal_handle_ = goal_handle;
    };
    goal_options.feedback_callback = [this](
                                       typename GoalHandle::SharedPtr /*goal_handle*/,
                                       const std::shared_ptr<const Feedback> feedback) {
      on_feedback_state_change_ = onFeedback(feedback);
      if (on_feedback_state_change_ == BT::NodeStatus::IDLE) {
        throw std::logic_error(context_.getFullyQualifiedTreeNodeName(this) + " - onFeedback() must not return IDLE.");
      }
      emitWakeUpSignal();
    };
    goal_options.result_callback = [this](const WrappedResult & result) {
      if (!goal_handle_)
        throw std::logic_error(
          context_.getFullyQualifiedTreeNodeName(this) + " - goal_handle_ is nullptr in result callback.");
      if (goal_handle_->get_goal_id() == result.goal_id) {
        if (result.code == rclcpp_action::ResultCode::CANCELED) {
          // The goal can only be canceled if the tree was halted. In this case, the tick callback won't be able to call
          // onResultReceived, so we must do this here. The returned status has no effect in this case.
          onResultReceived(result);
        }
        result_ = result;
        goal_handle_ = nullptr;  // Reset internal goal handle after result
        emitWakeUpSignal();
      }
    };

    future_goal_handle_ = action_client->async_send_goal(goal, goal_options);
    time_goal_sent_ = context_.getCurrentTime();
    return BT::NodeStatus::RUNNING;
  }

  if (status() == BT::NodeStatus::RUNNING) {
    std::unique_lock<std::mutex> lock(getMutex());

    // FIRST case: check if the goal request has a timeout as long as goal_response_received_ is false (Is set to true
    // as soon as a goal response is received)
    if (!goal_response_received_) {
      // See if we must time out
      if ((context_.getCurrentTime() - time_goal_sent_) > context_.registration_options_.request_timeout) {
        return check_status(onFailure(SEND_GOAL_TIMEOUT));
      }
      return BT::NodeStatus::RUNNING;
    } else if (future_goal_handle_.valid()) {
      // We noticed, that a goal response has just been received and have to prepare the next steps now
      future_goal_handle_ = {};  // Invalidate future since it's obsolete now and it indicates that we've done this step

      if (!goal_handle_) return check_status(onFailure(GOAL_REJECTED_BY_SERVER));
      RCLCPP_DEBUG(
        logger_, "%s - Goal accepted by server, waiting for result.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
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

template <class ActionT>
inline bool RosActionNode<ActionT>::createClient(const std::string & action_name)
{
  if (action_name.empty()) {
    throw exceptions::RosNodeError(
      context_.getFullyQualifiedTreeNodeName(this) +
      " - Argument action_name is empty when trying to create the client.");
  }

  // Check if the action with given name is already set up
  if (
    client_instance_ && action_name == client_instance_->name &&
    client_instance_->action_client->action_server_is_ready()) {
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
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;

  auto & registry = getRegistry();
  auto it = registry.find(action_client_key_);
  if (it == registry.end() || it->second.expired()) {
    client_instance_ = std::make_shared<ActionClientInstance>(node, group, action_name);
    registry.insert({action_client_key_, client_instance_});
    RCLCPP_DEBUG(
      logger_, "%s - Created client for action '%s'.", context_.getFullyQualifiedTreeNodeName(this).c_str(),
      action_name.c_str());
  } else {
    client_instance_ = it->second.lock();
  }

  bool found = client_instance_->action_client->wait_for_action_server(context_.registration_options_.wait_timeout);
  if (!found) {
    std::string msg = context_.getFullyQualifiedTreeNodeName(this) + " - Action server with name '" +
                      client_instance_->name + "' is not reachable.";
    if (context_.registration_options_.allow_unreachable) {
      RCLCPP_WARN_STREAM(logger_, msg);
    } else {
      RCLCPP_ERROR_STREAM(logger_, msg);
      throw exceptions::RosNodeError(msg);
    }
  }
  return found;
}

template <class ActionT>
inline std::string RosActionNode<ActionT>::getActionName() const
{
  if (client_instance_) return client_instance_->name;
  return "unkown";
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