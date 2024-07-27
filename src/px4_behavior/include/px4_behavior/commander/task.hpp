#pragma once

#include <chrono>
#include <px4_behavior/definitions.hpp>
#include <px4_behavior/commander/action_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace px4_behavior {

static constexpr std::chrono::milliseconds DEFAULT_VALUE_EXECUTION_INTERVAL{10};
static constexpr std::chrono::milliseconds DEFAULT_VALUE_FEEDBACK_INTERVAL{200};

enum class TaskStatus : uint8_t { RUNNING, SUCCESS, FAILURE };

template <class ActionT>
class TaskBase
{
   public:
    static constexpr char PARAM_NAME_FEEDBACK_INTERVAL[] = "feedback_interval_ms";

   protected:
    using Goal = typename ActionContext<ActionT>::Goal;
    using Feedback = typename ActionContext<ActionT>::Feedback;
    using Result = typename ActionContext<ActionT>::Result;
    using GoalHandle = typename ActionContext<ActionT>::GoalHandle;

    explicit TaskBase(const std::string& name,
                      rclcpp::Node::SharedPtr node_ptr,
                      std::shared_ptr<ActionContext<ActionT>> action_context_ptr,
                      std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                      std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
    explicit TaskBase(const std::string& name,
                      rclcpp::Node::SharedPtr node_ptr,
                      std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                      std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
    explicit TaskBase(const std::string& name,
                      const rclcpp::NodeOptions& options,
                      std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                      std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);

   private:
    /**
     *  TaskBase specific callbacks
     */
    virtual bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr);
    virtual void SetDefaultResult(std::shared_ptr<Result> result_ptr);
    virtual bool OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr);
    virtual TaskStatus CancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr);
    virtual TaskStatus ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                                               std::shared_ptr<Feedback> feedback_ptr,
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
    std::shared_ptr<ActionContext<ActionT>> action_context_ptr_;

   private:
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_ptr_;
    rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
    const std::chrono::milliseconds execution_timer_interval_;
    std::chrono::milliseconds feedback_interval_;
    std::chrono::steady_clock::time_point last_feedback_ts_;
};

template <class ActionT>
TaskBase<ActionT>::TaskBase(const std::string& name,
                            rclcpp::Node::SharedPtr node_ptr,
                            std::shared_ptr<ActionContext<ActionT>> action_context_ptr,
                            std::chrono::milliseconds execution_interval,
                            std::chrono::milliseconds feedback_interval)
    : node_ptr_{node_ptr},
      action_context_ptr_{action_context_ptr},
      execution_timer_interval_{execution_interval},
      feedback_interval_{feedback_interval}
{
#ifdef DEBUG_LOGGING
    // Set logging severity
    auto ret = rcutils_logging_set_logger_level(node_ptr_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
#endif

    using namespace std::placeholders;
    action_server_ptr_ =
        rclcpp_action::create_server<ActionT>(node_ptr_,
                                              name,
                                              std::bind(&TaskBase<ActionT>::handle_goal_, this, _1, _2),
                                              std::bind(&TaskBase<ActionT>::handle_cancel_, this, _1),
                                              std::bind(&TaskBase<ActionT>::handle_accepted_, this, _1));

    /**
     * Parameters
     */
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.description = "Rate at which this task publishes feedback.";
    node_ptr_->declare_parameter(
        PARAM_NAME_FEEDBACK_INTERVAL,
        std::chrono::duration_cast<std::chrono::milliseconds>(DEFAULT_VALUE_FEEDBACK_INTERVAL).count(),
        param_desc);
}

template <class ActionT>
TaskBase<ActionT>::TaskBase(const std::string& name,
                            rclcpp::Node::SharedPtr node_ptr,
                            std::chrono::milliseconds execution_interval,
                            std::chrono::milliseconds feedback_interval)
    : TaskBase{name,
               node_ptr,
               std::make_shared<ActionContext<ActionT>>(node_ptr->get_logger()),
               execution_interval,
               feedback_interval}
{}

template <class ActionT>
TaskBase<ActionT>::TaskBase(const std::string& name,
                            const rclcpp::NodeOptions& options,
                            std::chrono::milliseconds execution_interval,
                            std::chrono::milliseconds feedback_interval)
    : TaskBase{name,
               std::make_shared<rclcpp::Node>("task_" + name + "_node", options),
               execution_interval,
               feedback_interval}
{}

template <class ActionT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr TaskBase<ActionT>::get_node_base_interface() const
{
    return node_ptr_->get_node_base_interface();
}

template <class ActionT>
bool TaskBase<ActionT>::OnGoalRequest(std::shared_ptr<const Goal> goal_ptr)
{
    (void)goal_ptr;
    // Always accept goal by default
    return true;
}

template <class ActionT>
void TaskBase<ActionT>::SetDefaultResult(std::shared_ptr<Result> result_ptr)
{
    // By default, the result is initialized using the default values specified in the action message definition.
    (void)result_ptr;
}

template <class ActionT>
bool TaskBase<ActionT>::OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr)
{
    (void)goal_ptr;
    (void)result_ptr;
    // Always accept cancel request by default
    return true;
}

template <class ActionT>
TaskStatus TaskBase<ActionT>::CancelGoal(std::shared_ptr<const Goal> goal_ptr,
                                                     std::shared_ptr<Result> result_ptr)
{
    (void)goal_ptr;
    (void)result_ptr;
    // Do nothing by default
    return TaskStatus::SUCCESS;
}

template <class ActionT>
rclcpp_action::GoalResponse TaskBase<ActionT>::handle_goal_(const rclcpp_action::GoalUUID& uuid,
                                                            std::shared_ptr<const Goal> goal_ptr)
{
    if (action_context_ptr_->is_valid() && action_context_ptr_->goal_handle()->is_active()) {
        RCLCPP_DEBUG(node_ptr_->get_logger(),
                     "Goal %s was REJECTED because another one is still executing. ID of the executing goal: %s",
                     rclcpp_action::to_string(uuid).c_str(),
                     rclcpp_action::to_string(action_context_ptr_->goal_handle()->get_goal_id()).c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (OnGoalRequest(goal_ptr)) return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    RCLCPP_DEBUG(node_ptr_->get_logger(),
                 "Goal %s was REJECTED because OnGoalRequest() returned false",
                 rclcpp_action::to_string(uuid).c_str());
    return rclcpp_action::GoalResponse::REJECT;
}

template <class ActionT>
rclcpp_action::CancelResponse TaskBase<ActionT>::handle_cancel_(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
    (void)goal_handle_ptr;

    return OnCancelRequest(action_context_ptr_->goal_handle()->get_goal(), action_context_ptr_->result())
               ? rclcpp_action::CancelResponse::ACCEPT
               : rclcpp_action::CancelResponse::REJECT;
}

template <class ActionT>
void TaskBase<ActionT>::handle_accepted_(std::shared_ptr<GoalHandle> goal_handle_ptr)
{
    action_context_ptr_->SetUp(goal_handle_ptr);
    SetDefaultResult(action_context_ptr_->result());
    (void)goal_handle_ptr;  // action_context_ptr_ takes ownership of goal handle from now on

    // Create the timer that triggers the execution routine
    execution_timer_ptr_ =
        node_ptr_->create_wall_timer(execution_timer_interval_,
                                     [this, goal_ptr = action_context_ptr_->goal_handle()->get_goal()]() {
                                         this->execution_timer_callback_(goal_ptr);
                                     });

    // Resetting this timestamp here ensures that we wait for one interval before publishing feedback for the first time
    last_feedback_ts_ = std::chrono::steady_clock::now();
}

template <class ActionT>
void TaskBase<ActionT>::execution_timer_callback_(std::shared_ptr<const Goal> goal_ptr)
{
    // Cancel timer when goal has terminated
    if (!action_context_ptr_->goal_handle()->is_active()) {
        execution_timer_ptr_->cancel();
        return;
    }

    // Check if canceling
    if (action_context_ptr_->goal_handle()->is_canceling()) {
        switch (CancelGoal(goal_ptr, action_context_ptr_->result())) {
            case TaskStatus::RUNNING:
                return;
            case TaskStatus::SUCCESS:
                action_context_ptr_->Cancel();
                return;
            case TaskStatus::FAILURE:
                action_context_ptr_->Abort();
                return;
        }
    }
    else {
        switch (ExecuteGoal(goal_ptr, action_context_ptr_->feedback(), action_context_ptr_->result())) {
            case TaskStatus::RUNNING:
                break;
            case TaskStatus::SUCCESS:
                action_context_ptr_->Succeed();
                return;
            case TaskStatus::FAILURE:
                action_context_ptr_->Abort();
                return;
        }

        // Publish feedback
        feedback_interval_ = std::chrono::milliseconds(node_ptr_->get_parameter(PARAM_NAME_FEEDBACK_INTERVAL).as_int());
        if (feedback_interval_ < (std::chrono::steady_clock::now() - last_feedback_ts_)) {
            action_context_ptr_->PublishFeedback();
            last_feedback_ts_ = std::chrono::steady_clock::now();
        }
    }
}

}  // namespace px4_behavior
