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
#include "auto_apms_core/action_context.hpp"
#include "auto_apms_core/logging.hpp"

namespace auto_apms_core
{

/**
 * @defgroup auto_apms_core AutoAPMS - Core
 * @brief Fundamental base classes and utility functions.
 */

/**
 * @brief Status of the auto_apms_core::Task execution process.
 * @ingroup auto_apms_core
 */
enum class TaskStatus : uint8_t
{
  RUNNING,
  SUCCESS,
  FAILURE
};

/**
 * @brief Generic base class for robot actions.
 *
 * A auto_apms_core::Task is esentially a wrapper for a rclcpp_action::Server providing convenient extension points.
 *
 * @ingroup auto_apms_core
 */
template <typename ActionT>
class Task
{
public:
  using Status = TaskStatus;
  using ActionContextType = ActionContext<ActionT>;
  using Goal = typename ActionContextType::Goal;
  using Feedback = typename ActionContextType::Feedback;
  using Result = typename ActionContextType::Result;
  using GoalHandle = typename ActionContextType::GoalHandle;

  static constexpr std::chrono::milliseconds DEFAULT_VALUE_EXECUTION_INTERVAL{ 10 };
  static constexpr std::chrono::milliseconds DEFAULT_VALUE_FEEDBACK_INTERVAL{ 200 };
  static constexpr char PARAM_NAME_FEEDBACK_INTERVAL[] = "feedback_interval_ms";

  explicit Task(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                std::shared_ptr<ActionContextType> action_context_ptr,
                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
  explicit Task(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
  explicit Task(const std::string& name, const rclcpp::NodeOptions& options,
                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);

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
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_ptr_;
  rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
  const std::chrono::milliseconds execution_timer_interval_;
  std::chrono::steady_clock::time_point last_feedback_ts_;
};

template <class ActionT>
Task<ActionT>::Task(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                    std::shared_ptr<ActionContextType> action_context_ptr, std::chrono::milliseconds execution_interval,
                    std::chrono::milliseconds feedback_interval)
  : node_ptr_{ node_ptr }, action_context_ptr_{ action_context_ptr }, execution_timer_interval_{ execution_interval }
{
  exposeToDebugLogging(node_ptr_->get_logger());

  using namespace std::placeholders;
  action_server_ptr_ = rclcpp_action::create_server<ActionT>(
      node_ptr_, name, std::bind(&Task<ActionT>::handle_goal_, this, _1, _2),
      std::bind(&Task<ActionT>::handle_cancel_, this, _1), std::bind(&Task<ActionT>::handle_accepted_, this, _1));

  /**
   * Parameters
   */
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Rate at which this task publishes feedback.";
  node_ptr_->declare_parameter(PARAM_NAME_FEEDBACK_INTERVAL, feedback_interval.count(), param_desc);
}

template <class ActionT>
Task<ActionT>::Task(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                    std::chrono::milliseconds execution_interval, std::chrono::milliseconds feedback_interval)
  : Task{ name, node_ptr, std::make_shared<ActionContextType>(node_ptr->get_logger()), execution_interval,
          feedback_interval }
{
}

template <class ActionT>
Task<ActionT>::Task(const std::string& name, const rclcpp::NodeOptions& options,
                    std::chrono::milliseconds execution_interval, std::chrono::milliseconds feedback_interval)
  : Task{ name, std::make_shared<rclcpp::Node>("task_" + name + "_node", options), execution_interval,
          feedback_interval }
{
}

template <class ActionT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Task<ActionT>::get_node_base_interface() const
{
  return node_ptr_->get_node_base_interface();
}

template <class ActionT>
bool Task<ActionT>::onGoalRequest(std::shared_ptr<const Goal> goal_ptr)
{
  (void)goal_ptr;
  // Always accept goal by default
  return true;
}

template <class ActionT>
void Task<ActionT>::setInitialResult(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr)
{
  // By default, the result is initialized using the default values specified in the action message definition.
  (void)goal_ptr;
  (void)result_ptr;
}

template <class ActionT>
bool Task<ActionT>::onCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr)
{
  (void)goal_ptr;
  (void)result_ptr;
  // Always accept cancel request by default
  return true;
}

template <class ActionT>
TaskStatus Task<ActionT>::cancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr)
{
  (void)goal_ptr;
  (void)result_ptr;
  // Do nothing by default
  return TaskStatus::SUCCESS;
}

template <class ActionT>
rclcpp_action::GoalResponse Task<ActionT>::handle_goal_(const rclcpp_action::GoalUUID& uuid,
                                                        std::shared_ptr<const Goal> goal_ptr)
{
  if (action_context_ptr_->isValid() && action_context_ptr_->getGoalHandlePtr()->is_active())
  {
    RCLCPP_DEBUG(node_ptr_->get_logger(),
                 "Goal %s was REJECTED because another one is still executing. ID of the executing goal: %s",
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
rclcpp_action::CancelResponse Task<ActionT>::handle_cancel_(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
  (void)goal_handle_ptr;

  return onCancelRequest(action_context_ptr_->getGoalHandlePtr()->get_goal(), action_context_ptr_->getResultPtr()) ?
             rclcpp_action::CancelResponse::ACCEPT :
             rclcpp_action::CancelResponse::REJECT;
}

template <class ActionT>
void Task<ActionT>::handle_accepted_(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
  action_context_ptr_->setUp(goal_handle_ptr);
  const auto goal_ptr = action_context_ptr_->getGoalHandlePtr()->get_goal();
  setInitialResult(goal_ptr, action_context_ptr_->getResultPtr());
  (void)goal_handle_ptr;  // action_context_ptr_ takes ownership of goal handle from now on

  // Create the timer that triggers the execution routine
  execution_timer_ptr_ = node_ptr_->create_wall_timer(
      execution_timer_interval_, [this, goal_ptr]() { this->execution_timer_callback_(goal_ptr); });

  // Ensure that feedback is published already at the first cycle
  const std::chrono::milliseconds feedback_interval{ node_ptr_->get_parameter(PARAM_NAME_FEEDBACK_INTERVAL).as_int() };
  last_feedback_ts_ = std::chrono::steady_clock::now() - feedback_interval;
}

template <class ActionT>
void Task<ActionT>::execution_timer_callback_(std::shared_ptr<const Goal> goal_ptr)
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
      case TaskStatus::RUNNING:
        return;
      case TaskStatus::SUCCESS:
        action_context_ptr_->cancel();
        return;
      case TaskStatus::FAILURE:
        action_context_ptr_->abort();
        return;
    }
  }
  else
  {
    const auto ret = executeGoal(goal_ptr, action_context_ptr_->getFeedbackPtr(), action_context_ptr_->getResultPtr());

    // Publish feedback
    const std::chrono::milliseconds feedback_interval{
      node_ptr_->get_parameter(PARAM_NAME_FEEDBACK_INTERVAL).as_int()
    };
    if (feedback_interval <= (std::chrono::steady_clock::now() - last_feedback_ts_))
    {
      action_context_ptr_->publishFeedback();
      last_feedback_ts_ = std::chrono::steady_clock::now();
    }

    switch (ret)
    {
      case TaskStatus::RUNNING:
        break;
      case TaskStatus::SUCCESS:
        action_context_ptr_->succeed();
        return;
      case TaskStatus::FAILURE:
        action_context_ptr_->abort();
        return;
    }
  }
}

}  // namespace auto_apms_core
