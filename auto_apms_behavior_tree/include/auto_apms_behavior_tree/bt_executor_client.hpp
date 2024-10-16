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
#include <future>

#include "auto_apms_behavior_tree/behavior_tree.hpp"
#include "auto_apms_interfaces/action/launch_bt_executor.hpp"
#include "auto_apms_interfaces/srv/upload_behavior_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace auto_apms_behavior_tree {

using LaunchExecutorAction = auto_apms_interfaces::action::LaunchBTExecutor;
using UploadBehaviorTreeService = auto_apms_interfaces::srv::UploadBehaviorTree;
using ExecutionResultSharedPtr = std::shared_ptr<rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::WrappedResult>;

class BTExecutorClient
{
    const std::chrono::seconds WAIT_FOR_SERVER_TIMEOUT{3};
    const std::chrono::seconds UPLOAD_RESPONSE_TIMEOUT{5};
    const std::chrono::seconds LAUNCH_GOAL_RESPONSE_TIMEOUT{3};
    const std::chrono::seconds CANCEL_RESPONSE_TIMEOUT{1};

   public:
    BTExecutorClient(rclcpp::Node& node, const std::string& executor_name);

    /**
     * @brief Upload behavior tree to an executor.
     *
     * This method is synchronous, meaning that it blocks until upload result is received.
     *
     * @param resource Behavior tree resource.
     * @param main_tree_id ID of the tree to be created on upload. Empty if main_tree_to_execute XML attribute should be
     * used to determine which tree is to be created.
     * @return True on successful upload, false otherwise.
     */
    bool UploadBehaviorTree(const BehaviorTreeResource& resource, const std::string& main_tree_id = "");

    /**
     * @brief Upload behavior tree to an executor.
     *
     * This method is synchronous, meaning that it blocks until upload result is received.
     *
     * @param xml_data The XML string containing the trees to register.
     * @param main_tree_id ID of the tree to be created on upload. Empty if main_tree_to_execute XML attribute should be
     * used to determine which tree is to be created.
     * @return True on successful upload, false otherwise.
     */
    bool UploadBehaviorTree(std::string xml_data, const std::string& main_tree_id = "");

    /**
     * @brief Request to launch a behavoir tree executor.
     *
     * @return Shared future that completes when the process terminates, holding the result. Is nullptr if request
     * failed.
     */
    std::shared_future<ExecutionResultSharedPtr> RequestLaunch();

    /**
     * @brief Request to cancel the current behavior execution process.
     *
     * This method is synchronous, meaning that it blocks until the cancelation result is received.
     *
     * @return True if execution was cancelled successfully, false otherwise.
     */
    bool RequestCancelation();

    bool has_launched();
    const std::string& root_tree_id();
    const std::string& running_tree_action_name();
    double running_tree_action_timestamp();

   private:
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ptr_;
    const rclcpp::Logger logger_;
    const std::string executor_name_;
    const std::string upload_service_name_;
    const std::string launch_executor_action_name_;
    rclcpp::Client<UploadBehaviorTreeService>::SharedPtr upload_client_ptr_;
    rclcpp_action::Client<LaunchExecutorAction>::SharedPtr launch_client_ptr_;
    rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::SharedPtr launch_executor_goal_handle_{nullptr};

    std::string root_tree_id_;
    std::string running_tree_action_name_;
    double running_tree_action_timestamp_{0};
};

}  // namespace auto_apms_behavior_tree
