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

#include "action_msgs/srv/cancel_goal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace px4_behavior {

template <typename ActionT>
class ActionClientWrapper
{
   public:
    using Goal = typename ActionT::Goal;
    using SendGoalOptions = typename rclcpp_action::Client<ActionT>::SendGoalOptions;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
    using ClientResultSharedPtr = std::shared_ptr<typename ClientGoalHandle::WrappedResult>;

    ActionClientWrapper(rclcpp::Node& node, const std::string& action_name);

    /**
     * @brief Send a goal to the action and synchronously wait for the response.
     *
     * @param goal Goal of the action that will be sent to the server
     * @param options Goal options to be forwarded
     * @param server_timeout Timeout for waiting for the action server to be discovered
     * @param resonse_timeout Timeout for waiting for a goal response from the server
     * @return Shared future that completes when the action finishes, holding the result. The result is `nullptr` if
     * sending the goal has failed.
     */
    std::shared_future<ClientResultSharedPtr> SyncSendGoal(
        const Goal& goal = Goal{},
        const SendGoalOptions& options = SendGoalOptions{},
        const std::chrono::seconds server_timeout = std::chrono::seconds{3},
        const std::chrono::seconds response_timeout = std::chrono::seconds{3});

    /**
     * @brief Request to cancel the most recent goal.
     *
     * This method is synchronous, meaning that it blocks until the cancelation result is received.
     *
     * @param response_timeout Timeout for waiting for a cancel response from the server
     * @return `true` if the last goal was cancelled successfully, `false` if cancelation failed.
     */
    bool CancelLastGoal(const std::chrono::seconds response_timeout = std::chrono::seconds{3});

    /**
     * @brief Get the goal handle of the currently active goal.
     *
     * @return Shared pointer of the active goal handle or `nullptr`, if there is no active goal.
     */
    typename ClientGoalHandle::SharedPtr active_goal_handle();

   private:
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ptr_;
    const rclcpp::Logger logger_;
    const std::string action_name_;
    typename rclcpp_action::Client<ActionT>::SharedPtr client_ptr_;
    typename ClientGoalHandle::SharedPtr goal_handle_{nullptr};
};

template <typename ActionT>
ActionClientWrapper<ActionT>::ActionClientWrapper(rclcpp::Node& node, const std::string& action_name)
    : node_base_interface_ptr_{node.get_node_base_interface()}, logger_{node.get_logger()}, action_name_{action_name}
{
    client_ptr_ = rclcpp_action::create_client<ActionT>(&node, action_name_);
}

template <typename ActionT>
std::shared_future<typename ActionClientWrapper<ActionT>::ClientResultSharedPtr>
ActionClientWrapper<ActionT>::SyncSendGoal(const Goal& goal,
                                           const SendGoalOptions& options,
                                           const std::chrono::seconds server_timeout,
                                           const std::chrono::seconds response_timeout)
{
    auto promise_ptr = std::make_shared<std::promise<ClientResultSharedPtr>>();
    if (!client_ptr_->wait_for_action_server(server_timeout)) {
        RCLCPP_WARN(logger_, "Action server '%s' is not available", action_name_.c_str());
        promise_ptr->set_value(nullptr);
        return promise_ptr->get_future();
    }

    SendGoalOptions _options;
    _options.goal_response_callback = options.goal_response_callback;
    _options.feedback_callback = options.feedback_callback;
    _options.result_callback = [this, promise_ptr, options](const typename ClientGoalHandle::WrappedResult& wr) {
        if (options.result_callback) options.result_callback(wr);
        promise_ptr->set_value(std::make_shared<typename ClientGoalHandle::WrappedResult>(wr));
        goal_handle_ = nullptr;
    };

    auto goal_response_future = client_ptr_->async_send_goal(goal, _options);

    switch (rclcpp::spin_until_future_complete(node_base_interface_ptr_, goal_response_future, response_timeout)) {
        case rclcpp::FutureReturnCode::SUCCESS:
            break;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_WARN(logger_, "Goal response timeout");
            promise_ptr->set_value(nullptr);
            return promise_ptr->get_future();
        default:
            RCLCPP_WARN(logger_, "No goal response");
            promise_ptr->set_value(nullptr);
            return promise_ptr->get_future();
    }

    auto client_goal_handle_ptr = goal_response_future.get();
    if (!client_goal_handle_ptr) {
        RCLCPP_WARN(logger_, "Goal was rejected");
        promise_ptr->set_value(nullptr);
        return promise_ptr->get_future();
    }

    goal_handle_ = client_goal_handle_ptr;
    return promise_ptr->get_future();
}

template <typename ActionT>
bool ActionClientWrapper<ActionT>::CancelLastGoal(const std::chrono::seconds response_timeout)
{
    if (goal_handle_) {
        auto cancel_response_future = client_ptr_->async_cancel_goal(goal_handle_);
        switch (
            rclcpp::spin_until_future_complete(node_base_interface_ptr_, cancel_response_future, response_timeout)) {
            case rclcpp::FutureReturnCode::SUCCESS:
                break;
            case rclcpp::FutureReturnCode::TIMEOUT:
                RCLCPP_WARN(logger_, "Cancel response timeout");
                return false;
            default:
                RCLCPP_WARN(logger_, "No cancel response");
                return false;
        }

        // Evaluate the response message
        switch (cancel_response_future.get()->return_code) {
            case action_msgs::srv::CancelGoal::Response::ERROR_NONE:
                return true;
            case action_msgs::srv::CancelGoal::Response::ERROR_REJECTED:
                RCLCPP_WARN(logger_, "Cancel response ERROR_REJECTED");
                break;
            case action_msgs::srv::CancelGoal::Response::ERROR_UNKNOWN_GOAL_ID:
                RCLCPP_WARN(logger_, "Cancel response ERROR_UNKNOWN_GOAL_ID");
                break;
            case action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED:
                RCLCPP_WARN(logger_, "Cancel response ERROR_GOAL_TERMINATED");
                break;
        }
        return false;
    }
    RCLCPP_WARN(logger_, "Cannot cancel because goal handle is nullptr");
    return false;
}

template <typename ActionT>
typename ActionClientWrapper<ActionT>::ClientGoalHandle::SharedPtr ActionClientWrapper<ActionT>::active_goal_handle()
{
    return goal_handle_;
}

}  // namespace px4_behavior
