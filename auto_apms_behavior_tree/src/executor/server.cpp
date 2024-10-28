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

#include "auto_apms_behavior_tree/executor/server.hpp"

#include <functional>

#include "auto_apms_behavior_tree/constants.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

BTExecutorServer::BTExecutorServer(const std::string& name, rclcpp::NodeOptions options)
  : BTExecutorBase{ std::make_shared<rclcpp::Node>(name, options) }
  , logger_{ node()->get_logger() }
  , run_action_context_{ logger_ }
  , command_action_context_{ logger_ }
{
  using namespace std::placeholders;
  run_action_ptr_ = rclcpp_action::create_server<RunActionContext::Type>(
      node(), name + BT_EXECUTOR_RUN_ACTION_NAME_SUFFIX, std::bind(&BTExecutorServer::handle_run_goal_, this, _1, _2),
      std::bind(&BTExecutorServer::handle_run_cancel_, this, _1),
      std::bind(&BTExecutorServer::handle_run_accept_, this, _1));

  command_action_ptr_ = rclcpp_action::create_server<CommandActionContext::Type>(
      node(), name + BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX,
      std::bind(&BTExecutorServer::handle_command_goal_, this, _1, _2),
      std::bind(&BTExecutorServer::handle_command_cancel_, this, _1),
      std::bind(&BTExecutorServer::handle_command_accept_, this, _1));
}

rclcpp_action::GoalResponse BTExecutorServer::handle_run_goal_(const rclcpp_action::GoalUUID& uuid,
                                                               std::shared_ptr<const RunActionContext::Goal> goal_ptr)
{
  // Reject if a tree is already executing
  if (GetExecutionState() != ExecutionState::IDLE)
  {
    RCLCPP_WARN(logger_, "Goal %s was REJECTED because a tree with ID '%s' is already running.",
                rclcpp_action::to_string(uuid).c_str(), GetTreeName().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  try
  {
    TreeResource::FromString(goal_ptr->tree_identity);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger_, "Goal %s was REJECTED because no tree resource could be found using identity string '%s'.",
                rclcpp_action::to_string(uuid).c_str(), goal_ptr->tree_identity.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
BTExecutorServer::handle_run_cancel_(std::shared_ptr<RunActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  set_control_command(ControlCommand::HALT);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BTExecutorServer::handle_run_accept_(std::shared_ptr<RunActionContext::GoalHandle> goal_handle_ptr)
{
  run_action_context_.SetUp(goal_handle_ptr);
}

rclcpp_action::GoalResponse BTExecutorServer::handle_command_goal_(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CommandActionContext::Goal> goal_ptr)
{
  return rclcpp_action::GoalResponse();
}

rclcpp_action::CancelResponse
BTExecutorServer::handle_command_cancel_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr)
{
  return rclcpp_action::CancelResponse();
}

void BTExecutorServer::handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr)
{
  command_action_context_.SetUp(goal_handle_ptr);
}

}  // namespace auto_apms_behavior_tree