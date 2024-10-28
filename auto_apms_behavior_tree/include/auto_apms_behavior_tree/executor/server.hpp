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

#include "auto_apms_behavior_tree/executor/executor.hpp"
#include "auto_apms_core/action_context.hpp"
#include "auto_apms_interfaces/action/bt_executor_command.hpp"
#include "auto_apms_interfaces/action/run_behavior_tree.hpp"

namespace auto_apms_behavior_tree
{

class BTExecutorServer : public BTExecutorBase
{
public:
  using RunActionContext = auto_apms_core::ActionContext<auto_apms_interfaces::action::RunBehaviorTree>;
  using CommandActionContext = auto_apms_core::ActionContext<auto_apms_interfaces::action::BTExecutorCommand>;

  BTExecutorServer(const std::string& name, rclcpp::NodeOptions options);

private:
  /// @cond
  rclcpp_action::GoalResponse handle_run_goal_(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const RunActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_run_cancel_(std::shared_ptr<RunActionContext::GoalHandle> goal_handle_ptr);
  void handle_run_accept_(std::shared_ptr<RunActionContext::GoalHandle> goal_handle_ptr);

  rclcpp_action::GoalResponse handle_command_goal_(const rclcpp_action::GoalUUID& uuid,
                                                   std::shared_ptr<const CommandActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse
  handle_command_cancel_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  void handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  /// @endcond

  const rclcpp::Logger logger_;
  rclcpp_action::Server<RunActionContext::Type>::SharedPtr run_action_ptr_;
  RunActionContext run_action_context_;
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  CommandActionContext command_action_context_;
};

}  // namespace auto_apms_behavior_tree