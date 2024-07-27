#pragma once

#include <rclcpp_action/rclcpp_action.hpp>

namespace px4_behavior {

template <class ActionT>
class ActionContext
{
   public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    ActionContext(const rclcpp::Logger& logger);

    void SetUp(std::shared_ptr<GoalHandle> goal_handle_ptr);
    void PublishFeedback();
    void Succeed();
    void Cancel();
    void Abort();

    std::shared_ptr<GoalHandle> goal_handle();
    bool is_valid() { return (bool)goal_handle_ptr_; }
    std::shared_ptr<Feedback> feedback() { return feedback_ptr_; }
    std::shared_ptr<Result> result() { return result_ptr_; }

   private:
    const rclcpp::Logger logger_;
    std::shared_ptr<GoalHandle> goal_handle_ptr_;
    const std::shared_ptr<Feedback> feedback_ptr_{std::make_shared<Feedback>()};
    const std::shared_ptr<Result> result_ptr_{std::make_shared<Result>()};
};

template <class ActionT>
ActionContext<ActionT>::ActionContext(const rclcpp::Logger& logger) : logger_{logger}
{}

template <class ActionT>
void ActionContext<ActionT>::SetUp(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
    goal_handle_ptr_ = goal_handle_ptr;
    *feedback_ptr_ = Feedback{};
    *result_ptr_ = Result{};
}

template <class ActionT>
void ActionContext<ActionT>::PublishFeedback()
{
    if (!goal_handle_ptr_) {
        RCLCPP_FATAL(logger_, "The node tried to publish feedback on a goal, but no goal handle was set up");
        throw std::runtime_error("goal_handle_ptr_ is nullptr");
    }
    if (goal_handle_ptr_->is_executing()) { goal_handle_ptr_->publish_feedback(feedback_ptr_); }
    else {
        RCLCPP_WARN(logger_,
                    "The node tried to publish feedback on goal %s which is not executing. Ignoring ...",
                    rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
    }
}

template <class ActionT>
void ActionContext<ActionT>::Succeed()
{
    if (!goal_handle_ptr_) {
        RCLCPP_FATAL(logger_, "The node tried to succeed the goal, but no goal handle was set up");
        throw std::runtime_error("goal_handle_ptr_ is nullptr");
    }
    RCLCPP_DEBUG(logger_, "Goal %s succeeded", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
    goal_handle_ptr_->succeed(result_ptr_);
}

template <class ActionT>
void ActionContext<ActionT>::Cancel()
{
    if (!goal_handle_ptr_) {
        RCLCPP_FATAL(logger_, "The node tried to cancel the goal, but no goal handle was set up");
        throw std::runtime_error("goal_handle_ptr_ is nullptr");
    }
    RCLCPP_DEBUG(logger_, "Goal %s was canceled", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
    goal_handle_ptr_->canceled(result_ptr_);
}

template <class ActionT>
void ActionContext<ActionT>::Abort()
{
    if (!goal_handle_ptr_) {
        RCLCPP_FATAL(logger_, "The node tried to abort the goal, but no goal handle was set up");
        throw std::runtime_error("goal_handle_ptr_ is nullptr");
    }
    RCLCPP_DEBUG(logger_, "Goal %s was aborted", rclcpp_action::to_string(goal_handle_ptr_->get_goal_id()).c_str());
    goal_handle_ptr_->abort(result_ptr_);
}

template <class ActionT>
std::shared_ptr<typename ActionContext<ActionT>::GoalHandle> ActionContext<ActionT>::goal_handle()
{
    if (!goal_handle_ptr_) {
        RCLCPP_FATAL(logger_,
                     "The node tried to access the goal handle calling ActionContext::goal_handle() but no goal handle "
                     "was set up");
        throw std::runtime_error("goal_handle_ptr_ is nullptr");
    }
    return goal_handle_ptr_;
}

}  // namespace px4_behavior