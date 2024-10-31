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

#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_core
{

template <typename ActionT>
class ActionContext
{
public:
  using Type = ActionT;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  ActionContext(const rclcpp::Logger& logger);

  void setUp(std::shared_ptr<GoalHandle> goal_handle_ptr);

  void publishFeedback();

  void succeed();

  void cancel();

  void abort();

  void releaseGoalHandle();

  std::shared_ptr<GoalHandle> getGoalHandlePtr();

  bool isValid();

  std::shared_ptr<Feedback> getFeedbackPtr();

  std::shared_ptr<Result> getResultPtr();

private:
  const rclcpp::Logger logger_;
  std::shared_ptr<GoalHandle> goal_handle_ptr_;
  const std::shared_ptr<Feedback> feedback_ptr_{ std::make_shared<Feedback>() };
  const std::shared_ptr<Result> result_ptr_{ std::make_shared<Result>() };
};

/**
 * ------------------ DEFINITIONS ------------------
 */

template <class ActionT>
ActionContext<ActionT>::ActionContext(const rclcpp::Logger& logger) : logger_{ logger }
{
}

template <class ActionT>
void ActionContext<ActionT>::setUp(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
  goal_handle_ptr_ = goal_handle_ptr;
  *feedback_ptr_ = Feedback{};
  *result_ptr_ = Result{};
}

template <class ActionT>
void ActionContext<ActionT>::publishFeedback()
{
  if (!goal_handle_ptr_)
  {
    RCLCPP_FATAL(logger_, "Tried to publish feedback on a goal, but no goal handle was set up.");
    throw std::runtime_error("goal_handle_ptr_ is nullptr.");
  }
  if (goal_handle_ptr_->is_executing())
  {
    goal_handle_ptr_->publish_feedback(feedback_ptr_);
  }
  else
  {
    RCLCPP_WARN(logger_, "The node tried to publish feedback on goal %s which is not executing. Ignoring ...",
                rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
  }
}

template <class ActionT>
void ActionContext<ActionT>::succeed()
{
  if (!goal_handle_ptr_)
  {
    RCLCPP_FATAL(logger_, "Tried to succeed the goal, but no goal handle was set up.");
    throw std::runtime_error("goal_handle_ptr_ is nullptr.");
  }
  RCLCPP_DEBUG(logger_, "Goal %s succeeded.", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
  goal_handle_ptr_->succeed(result_ptr_);
}

template <class ActionT>
void ActionContext<ActionT>::cancel()
{
  if (!goal_handle_ptr_)
  {
    RCLCPP_FATAL(logger_, "Tried to cancel the goal, but no goal handle was set up.");
    throw std::runtime_error("goal_handle_ptr_ is nullptr.");
  }
  RCLCPP_DEBUG(logger_, "Goal %s was canceled.", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
  goal_handle_ptr_->canceled(result_ptr_);
}

template <class ActionT>
void ActionContext<ActionT>::abort()
{
  if (!goal_handle_ptr_)
  {
    RCLCPP_FATAL(logger_, "Tried to abort the goal, but no goal handle was set up.");
    throw std::runtime_error("goal_handle_ptr_ is nullptr.");
  }
  RCLCPP_DEBUG(logger_, "Goal %s was aborted.", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
  goal_handle_ptr_->abort(result_ptr_);
}

template <typename ActionT>
inline void ActionContext<ActionT>::releaseGoalHandle()
{
  goal_handle_ptr_.reset();
}

template <class ActionT>
std::shared_ptr<typename ActionContext<ActionT>::GoalHandle> ActionContext<ActionT>::getGoalHandlePtr()
{
  if (!goal_handle_ptr_)
  {
    RCLCPP_FATAL(logger_,
                 "Tried to access the goal handle calling ActionContext::goal_handle() but no goal handle was set up.");
    throw std::runtime_error("goal_handle_ptr_ is nullptr.");
  }
  return goal_handle_ptr_;
}

template <typename ActionT>
inline bool ActionContext<ActionT>::isValid()
{
  return !!goal_handle_ptr_;
}

template <typename ActionT>
inline std::shared_ptr<typename ActionContext<ActionT>::Feedback> ActionContext<ActionT>::getFeedbackPtr()
{
  return feedback_ptr_;
}

template <typename ActionT>
inline std::shared_ptr<typename ActionContext<ActionT>::Result> ActionContext<ActionT>::getResultPtr()
{
  return result_ptr_;
}

}  // namespace auto_apms_core
