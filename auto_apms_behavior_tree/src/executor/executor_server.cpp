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

#include "auto_apms_behavior_tree/executor/executor_server.hpp"

#include <functional>

#include "auto_apms_behavior_tree/definitions.hpp"
#include "auto_apms_behavior_tree/resource/tree_resource.hpp"

namespace auto_apms_behavior_tree
{

// clang-format off
const std::string BTExecutorServer::PARAM_NAME_BUILD_DIRECTOR = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_BUILD_DIRECTOR;
const std::string BTExecutorServer::PARAM_NAME_TICK_RATE = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE;
const std::string BTExecutorServer::PARAM_NAME_GROOT2_PORT = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT;
const std::string BTExecutorServer::PARAM_NAME_STATE_CHANGE_LOGGER = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER;
// clang-format off

BTExecutorServer::BTExecutorServer(const std::string& name, rclcpp::NodeOptions options)
  : BTExecutorBase(std::make_shared<rclcpp::Node>(name, options))
  , logger_(getNodePtr()->get_logger())
  , executor_param_listener_(getNodePtr())
  , node_manifest_param_listener_(getNodePtr())
  , run_action_context_(logger_)
  , command_action_context_(logger_)
{
  using namespace std::placeholders;
  run_action_ptr_ =
      rclcpp_action::create_server<RunActionContext::Type>(getNodePtr(), name + BT_EXECUTOR_RUN_ACTION_NAME_SUFFIX,
                                                           std::bind(&BTExecutorServer::handle_run_goal_, this, _1, _2),
                                                           std::bind(&BTExecutorServer::handle_run_cancel_, this, _1),
                                                           std::bind(&BTExecutorServer::handle_run_accept_, this, _1));

  command_action_ptr_ = rclcpp_action::create_server<CommandActionContext::Type>(
      getNodePtr(), name + BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX,
      std::bind(&BTExecutorServer::handle_command_goal_, this, _1, _2),
      std::bind(&BTExecutorServer::handle_command_cancel_, this, _1),
      std::bind(&BTExecutorServer::handle_command_accept_, this, _1));

  on_set_parameters_callback_handle_ =
      getNodePtr()->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        return this->on_set_parameters_callback_(parameters);
      });
}

rcl_interfaces::msg::SetParametersResult
BTExecutorServer::on_set_parameters_callback_(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto& p : parameters)
  {
    // if (!some_condition)
    // {
    //   result.successful = false;
    //   result.reason = "the reason it could not be allowed";
    // }
  }
  return result;
}

rclcpp_action::GoalResponse BTExecutorServer::handle_run_goal_(const rclcpp_action::GoalUUID& uuid,
                                                               std::shared_ptr<const RunActionContext::Goal> goal_ptr)
{
  // Reject if a tree is already executing
  if (getExecutionState() != ExecutionState::IDLE)
  {
    RCLCPP_WARN(logger_, "Goal %s was REJECTED because a tree with ID '%s' is already running.",
                rclcpp_action::to_string(uuid).c_str(), getTreeName().c_str());
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
  setControlCommand(ControlCommand::HALT);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BTExecutorServer::handle_run_accept_(std::shared_ptr<RunActionContext::GoalHandle> goal_handle_ptr)
{
  run_action_context_.setUp(goal_handle_ptr);
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
  command_action_context_.setUp(goal_handle_ptr);
}

}  // namespace auto_apms_behavior_tree