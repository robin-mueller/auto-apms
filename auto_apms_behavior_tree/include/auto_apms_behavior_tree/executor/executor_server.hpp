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

#include "auto_apms_behavior_tree/creator/tree_builder.hpp"
#include "auto_apms_behavior_tree/executor/executor.hpp"
#include "auto_apms_behavior_tree/resource/node_registration_class_loader.hpp"
#include "auto_apms_behavior_tree/resource/tree_creator_class_loader.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "auto_apms_util/action_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

class TreeExecutorServer : public TreeExecutor
{
public:
  using StartActionContext = auto_apms_util::ActionContext<auto_apms_interfaces::action::StartTreeExecutor>;
  using CommandActionContext = auto_apms_util::ActionContext<auto_apms_interfaces::action::CommandTreeExecutor>;

  static const std::string PARAM_NAME_TREE_BUILDER;
  static const std::string PARAM_NAME_TICK_RATE;
  static const std::string PARAM_NAME_GROOT2_PORT;
  static const std::string PARAM_NAME_STATE_CHANGE_LOGGER;
  inline static const std::string DEFAULT_NODE_NAME = "tree_executor";

  /**
   * @brief Constructor for TreeExecutorServer with custom name.
   * @param[in] name Name of the rclcpp::Node instance.
   * @param[in] options Options forwarded to rclcpp::Node constructor.
   */
  TreeExecutorServer(const std::string & name, const rclcpp::NodeOptions & options);

  /**
   * @brief Constructor for TreeExecutorServer with default name TreeExecutorServer::DEFAULT_NODE_NAME.
   * @param[in] options Options forwarded to rclcpp::Node constructor.
   */
  TreeExecutorServer(const rclcpp::NodeOptions & options);

private:
  virtual void prepareTreeBuilder(TreeBuilder & builder);

  CreateTreeCallback makeCreateTreeCallback(
    const std::string & tree_creator_name, const std::string & tree_creator_request,
    const NodeManifest & node_overrides = {});

protected:
  virtual bool onTick() override;

  virtual void onTermination(const ExecutionResult & result) override;

private:
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback_(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp_action::GoalResponse handle_start_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_start_cancel_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);
  void handle_start_accept_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);

  rclcpp_action::GoalResponse handle_command_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CommandActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_command_cancel_(
    std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  void handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);

  const rclcpp::Logger logger_;
  const executor_params::ParamListener executor_param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  TreeCreatorClassLoader tree_creator_loader_;
  std::shared_ptr<NodeRegistrationClassLoader> node_loader_ptr_;
  CreateTreeCallback create_tree_callback_;

  // Interface objects
  rclcpp_action::Server<StartActionContext::Type>::SharedPtr start_action_ptr_;
  StartActionContext start_action_context_;
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  CommandActionContext command_action_context_;
  rclcpp::TimerBase::SharedPtr command_timer_ptr_;
};

}  // namespace auto_apms_behavior_tree