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

#include "rclcpp/rclcpp.hpp"
#include "auto_apms_core/action_context.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
#include "auto_apms_behavior_tree/executor/executor.hpp"
#include "auto_apms_behavior_tree/node/node_manifest.hpp"
#include "auto_apms_behavior_tree/resource/tree_build_director_class_loader.hpp"

namespace auto_apms_behavior_tree
{

class BTExecutorServer : public BTExecutorBase
{
public:
  using StartActionContext = auto_apms_core::ActionContext<auto_apms_interfaces::action::StartTreeExecutor>;
  using CommandActionContext = auto_apms_core::ActionContext<auto_apms_interfaces::action::CommandTreeExecutor>;

  static const std::string PARAM_NAME_BUILD_DIRECTOR;
  static const std::string PARAM_NAME_TICK_RATE;
  static const std::string PARAM_NAME_GROOT2_PORT;
  static const std::string PARAM_NAME_STATE_CHANGE_LOGGER;
  inline static const std::string NODE_OVERRIDE_PARAMETERS_NAMESPACE = "node_override";
  inline static const std::string DEFAULT_NODE_NAME = "tree_executor";

  /**
   * @brief Constructor for BTExecutorServer with custom name.
   * @param[in] name Name of the rclcpp::Node instance.
   * @param[in] options Options forwarded to rclcpp::Node constructor.
   */
  BTExecutorServer(const std::string& name, const rclcpp::NodeOptions& options);

  /**
   * @brief Constructor for BTExecutorServer with default name BTExecutorServer::DEFAULT_NODE_NAME.
   * @param[in] options Options forwarded to rclcpp::Node constructor.
   */
  BTExecutorServer(const rclcpp::NodeOptions& options);

  CreateTreeCallback makeCreateTreeCallback(const std::string& tree_identity);

private:
  rcl_interfaces::msg::SetParametersResult
  on_set_parameters_callback_(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp_action::GoalResponse handle_start_goal_(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const StartActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_start_cancel_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);
  void handle_start_accept_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);

  rclcpp_action::GoalResponse handle_command_goal_(const rclcpp_action::GoalUUID& uuid,
                                                   std::shared_ptr<const CommandActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse
  handle_command_cancel_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  void handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);

  bool onTick() override;

  void onClose(const ExecutionResult& result) override;

  const rclcpp::Logger logger_;
  const executor_params::ParamListener executor_param_listener_;
  const NodeManifest::ParamListener node_override_param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  TreeBuildDirectorClassLoader tree_build_director_loader_;
  CreateTreeCallback create_tree_callback_;
  rclcpp_action::Server<StartActionContext::Type>::SharedPtr start_action_ptr_;
  StartActionContext start_action_context_;
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  CommandActionContext command_action_context_;
  rclcpp::TimerBase::SharedPtr command_timer_ptr_;
};

}  // namespace auto_apms_behavior_tree