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

#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree/resource/build_handler_loader.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/resource/node_registration_loader.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "auto_apms_util/action_context.hpp"
#include "executor_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

class TreeExecutorNodeOptions
{
public:
  TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options);

  TreeExecutorNodeOptions & enableScriptingEnumParameters(bool from_overrides, bool dynamic);

  TreeExecutorNodeOptions & enableBlackboardParameters(bool from_overrides, bool dynamic);

  rclcpp::NodeOptions getROSNodeOptions();

private:
  friend class TreeExecutorNode;

  rclcpp::NodeOptions ros_node_options_;
  bool scripting_enum_parameters_from_overrides_ = true;
  bool scripting_enum_parameters_dynamic_ = true;
  bool blackboard_parameters_from_overrides_ = true;
  bool blackboard_parameters_dynamic_ = true;
};

class TreeExecutorNode : public TreeExecutorBase
{
public:
  using Options = TreeExecutorNodeOptions;
  using ExecutorParameters = executor_params::Params;
  using ExecutorParameterListener = executor_params::ParamListener;
  using StartActionContext = auto_apms_util::ActionContext<auto_apms_interfaces::action::StartTreeExecutor>;
  using CommandActionContext = auto_apms_util::ActionContext<auto_apms_interfaces::action::CommandTreeExecutor>;
  using TreeBuilder = core::TreeBuilder;

  inline static const std::string PARAM_VALUE_NO_BUILD_HANDLER = "none";
  inline static const std::string SCRIPTING_ENUM_PARAM_PREFIX = "enum";
  inline static const std::string BLACKBOARD_PARAM_PREFIX = "bb";

  /**
   * @brief Constructor for TreeExecutorNode allowing to specify a default node name and executor options.
   * @param[in] name Name of the rclcpp::Node instance.
   * @param[in] executor_options Executor specific options. Simply pass a rclcpp::NodeOptions instance to use the
   * default options.
   */
  TreeExecutorNode(const std::string & name, TreeExecutorNodeOptions executor_options);

  /**
   * @brief Constructor for TreeExecutorNode with default name TreeExecutorNode::DEFAULT_NODE_NAME and default
   * TreeExecutorNodeOptions.
   * @param[in] options Options forwarded to rclcpp::Node constructor.
   */
  explicit TreeExecutorNode(rclcpp::NodeOptions options);

private:
  /* Virtual methods */

  virtual void setUpBuilder(TreeBuilder & builder);

protected:
  /* Utility methods */

  std::map<std::string, rclcpp::ParameterValue> getParameterValuesWithPrefix(const std::string & prefix);

  std::string stripPrefixFromParameterName(const std::string & prefix, const std::string & param_name);

  bool updateScriptingEnumsWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate = false);

  bool updateBlackboardWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, TreeBlackboard & bb, bool simulate = false);

  void loadBuildHandler(const std::string & name);

  TreeConstructor makeTreeConstructor(
    const std::string & build_handler_request, const std::string & root_tree_name,
    const core::NodeManifest & node_overrides = {});

private:
  /* Executor specific virtual overrides */

  virtual bool onTick() override final;

  virtual void onTermination(const ExecutionResult & result) override final;

  /* Internal callbacks */

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback_(
    const std::vector<rclcpp::Parameter> & parameters);

  void parameter_event_callback_(const rcl_interfaces::msg::ParameterEvent & event);

  rclcpp_action::GoalResponse handle_start_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_start_cancel_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);
  void handle_start_accept_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr);

  rclcpp_action::GoalResponse handle_command_goal_(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CommandActionContext::Goal> goal_ptr);
  rclcpp_action::CancelResponse handle_command_cancel_(
    std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);
  void handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr);

  const TreeExecutorNodeOptions executor_options_;
  const rclcpp::Logger logger_;
  ExecutorParameterListener executor_param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_ptr_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_ptr_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_event_callback_handle_ptr_;
  core::NodeRegistrationLoader::SharedPtr node_loader_ptr_;
  TreeBuildHandlerLoader::UniquePtr build_handler_loader_ptr_;
  TreeBuildHandler::UniquePtr build_handler_ptr_;
  std::string current_build_handler_name_;
  std::map<std::string, int> scripting_enum_buffer_;
  TreeConstructor tree_constructor_;

  // Interface objects
  rclcpp_action::Server<StartActionContext::Type>::SharedPtr start_action_ptr_;
  StartActionContext start_action_context_;
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  rclcpp::TimerBase::SharedPtr command_timer_ptr_;
};

}  // namespace auto_apms_behavior_tree