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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "auto_apms_util/action_context.hpp"
#include "auto_apms_util/logging.hpp"
#include "action_wrapper_params.hpp"

namespace auto_apms_util
{

/**
 * @defgroup auto_apms_util AutoAPMS - Utilities
 * @brief Fundamental helper classes and utility functions.
 */

/**
 * @brief Status of the auto_apms_util::ActionWrapper execution process.
 * @ingroup auto_apms_util
 */
enum class ActionStatus : uint8_t
{
  RUNNING,
  SUCCESS,
  FAILURE
};

extern const std::string ACTION_WRAPPER_PARAM_NAME_LOOP_RATE;
extern const std::string ACTION_WRAPPER_PARAM_NAME_FEEDBACK_RATE;

/**
 * @brief Generic base class for robot actions.
 *
 * A auto_apms_util::ActionWrapper is a wrapper for a rclcpp_action::Server providing convenient extension points.
 *
 * @ingroup auto_apms_util
 */
template <typename ActionT>
class ActionWrapper
{
public:
  using Params = action_wrapper_params::Params;
  using ParamListener = action_wrapper_params::ParamListener;
  using Status = ActionStatus;
  using ActionContextType = ActionContext<ActionT>;
  using Goal = typename ActionContextType::Goal;
  using Feedback = typename ActionContextType::Feedback;
  using Result = typename ActionContextType::Result;
  using GoalHandle = typename ActionContextType::GoalHandle;

  explicit ActionWrapper(const std::string& action_name, rclcpp::Node::SharedPtr node_ptr,
                         std::shared_ptr<ActionContextType> action_context_ptr);

  explicit ActionWrapper(const std::string& action_name, rclcpp::Node::SharedPtr node_ptr);

  explicit ActionWrapper(const std::string& action_name, const rclcpp::NodeOptions& options);

private:
  /**
   *  Implementation specific callbacks
   */

  virtual bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr);

  virtual void setInitialResult(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr);

  virtual bool onCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr);

  virtual Status cancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr);

  virtual Status executeGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
                             std::shared_ptr<Result> result_ptr) = 0;

  /**
   *  Action server callbacks
   */

  rclcpp_action::GoalResponse handle_goal_(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_cancel_(std::shared_ptr<GoalHandle> goal_handle_ptr);
  void handle_accepted_(std::shared_ptr<GoalHandle> goal_handle_ptr);

  void execution_timer_callback_(std::shared_ptr<const Goal> goal_ptr);

public:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  std::shared_ptr<ActionContextType> action_context_ptr_;

private:
  ParamListener param_listener_;
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_ptr_;
  rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
  rclcpp::Time last_feedback_ts_;
};

template <class ActionT>
ActionWrapper<ActionT>::ActionWrapper(const std::string& action_name, rclcpp::Node::SharedPtr node_ptr,
                                      std::shared_ptr<ActionContextType> action_context_ptr)
  : node_ptr_(node_ptr), action_context_ptr_(action_context_ptr), param_listener_(node_ptr)
{
  exposeToDebugLogging(node_ptr_->get_logger());

  using namespace std::placeholders;
  action_server_ptr_ = rclcpp_action::create_server<ActionT>(
      node_ptr_, action_name, std::bind(&ActionWrapper<ActionT>::handle_goal_, this, _1, _2),
      std::bind(&ActionWrapper<ActionT>::handle_cancel_, this, _1),
      std::bind(&ActionWrapper<ActionT>::handle_accepted_, this, _1));
}

template <class ActionT>
ActionWrapper<ActionT>::ActionWrapper(const std::string& action_name, rclcpp::Node::SharedPtr node_ptr)
  : ActionWrapper(action_name, node_ptr, std::make_shared<ActionContextType>(node_ptr->get_logger()))
{
}

template <class ActionT>
ActionWrapper<ActionT>::ActionWrapper(const std::string& action_name, const rclcpp::NodeOptions& options)
  : ActionWrapper(action_name, std::make_shared<rclcpp::Node>(action_name + "_node", options))
{
}

template <class ActionT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ActionWrapper<ActionT>::get_node_base_interface() const
{
  return node_ptr_->get_node_base_interface();
}

template <class ActionT>
bool ActionWrapper<ActionT>::onGoalRequest(std::shared_ptr<const Goal> /*goal_ptr*/)
{
  // Always accept goal by default
  return true;
}

template <class ActionT>
void ActionWrapper<ActionT>::setInitialResult(std::shared_ptr<const Goal> /*goal_ptr*/,
                                              std::shared_ptr<Result> /*result_ptr*/)
{
  // By default, the result is initialized using the default values specified in the action message definition.
}

template <class ActionT>
bool ActionWrapper<ActionT>::onCancelRequest(std::shared_ptr<const Goal> /*goal_ptr*/,
                                             std::shared_ptr<Result> /*result_ptr*/)
{
  // Always accept cancel request by default
  return true;
}

template <class ActionT>
ActionStatus ActionWrapper<ActionT>::cancelGoal(std::shared_ptr<const Goal> /*goal_ptr*/,
                                                std::shared_ptr<Result> /*result_ptr*/)
{
  // Do nothing by default
  return ActionStatus::SUCCESS;
}

template <class ActionT>
rclcpp_action::GoalResponse ActionWrapper<ActionT>::handle_goal_(const rclcpp_action::GoalUUID& uuid,
                                                                 std::shared_ptr<const Goal> goal_ptr)
{
  if (action_context_ptr_->isValid() && action_context_ptr_->getGoalHandlePtr()->is_active())
  {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "Goal %s was REJECTED: Goal %s is still executing.",
                 rclcpp_action::to_string(uuid).c_str(),
                 rclcpp_action::to_string(action_context_ptr_->getGoalHandlePtr()->get_goal_id()).c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (onGoalRequest(goal_ptr))
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  RCLCPP_DEBUG(node_ptr_->get_logger(), "Goal %s was REJECTED because onGoalRequest() returned false",
               rclcpp_action::to_string(uuid).c_str());
  return rclcpp_action::GoalResponse::REJECT;
}

template <class ActionT>
rclcpp_action::CancelResponse ActionWrapper<ActionT>::handle_cancel_(std::shared_ptr<GoalHandle> /*goal_handle_ptr*/)
{
  return onCancelRequest(action_context_ptr_->getGoalHandlePtr()->get_goal(), action_context_ptr_->getResultPtr()) ?
             rclcpp_action::CancelResponse::ACCEPT :
             rclcpp_action::CancelResponse::REJECT;
}

template <class ActionT>
void ActionWrapper<ActionT>::handle_accepted_(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
  action_context_ptr_->setUp(goal_handle_ptr);
  const auto goal_ptr = action_context_ptr_->getGoalHandlePtr()->get_goal();
  setInitialResult(goal_ptr, action_context_ptr_->getResultPtr());
  (void)goal_handle_ptr;  // action_context_ptr_ takes ownership of goal handle from now on

  // Create the timer that triggers the execution routine
  const Params& params = param_listener_.get_params();
  execution_timer_ptr_ =
      node_ptr_->create_wall_timer(std::chrono::duration<double>(params.loop_rate),
                                   [this, goal_ptr]() { this->execution_timer_callback_(goal_ptr); });

  // Ensure that feedback is published already at the first cycle
  const auto feedback_rate = rclcpp::Duration::from_seconds(params.feedback_rate);
  last_feedback_ts_ = node_ptr_->now() - feedback_rate;
}

template <class ActionT>
void ActionWrapper<ActionT>::execution_timer_callback_(std::shared_ptr<const Goal> goal_ptr)
{
  // Cancel timer when goal has terminated
  if (!action_context_ptr_->getGoalHandlePtr()->is_active())
  {
    execution_timer_ptr_->cancel();
    return;
  }

  // Check if canceling
  if (action_context_ptr_->getGoalHandlePtr()->is_canceling())
  {
    switch (cancelGoal(goal_ptr, action_context_ptr_->getResultPtr()))
    {
      case ActionStatus::RUNNING:
        return;
      case ActionStatus::SUCCESS:
        action_context_ptr_->cancel();
        return;
      case ActionStatus::FAILURE:
        action_context_ptr_->abort();
        return;
    }
  }
  else
  {
    const auto ret = executeGoal(goal_ptr, action_context_ptr_->getFeedbackPtr(), action_context_ptr_->getResultPtr());

    // Publish feedback
    const auto feedback_rate = rclcpp::Duration::from_seconds(param_listener_.get_params().feedback_rate);
    if ((node_ptr_->now() - last_feedback_ts_) > feedback_rate)
    {
      action_context_ptr_->publishFeedback();
      last_feedback_ts_ = node_ptr_->now();
    }

    switch (ret)
    {
      case ActionStatus::RUNNING:
        break;
      case ActionStatus::SUCCESS:
        action_context_ptr_->succeed();
        return;
      case ActionStatus::FAILURE:
        action_context_ptr_->abort();
        return;
    }
  }
}

}  // namespace auto_apms_util
