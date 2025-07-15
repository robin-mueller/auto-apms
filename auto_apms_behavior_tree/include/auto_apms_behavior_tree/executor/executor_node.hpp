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

#include "auto_apms_behavior_tree/build_handler/build_handler_loader.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree/executor_params.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_loader.hpp"
#include "auto_apms_interfaces/action/command_tree_executor.hpp"
#include "auto_apms_interfaces/action/start_tree_executor.hpp"
#include "auto_apms_util/action_context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief Configuration options for TreeExecutorNode.
 *
 * This allows to hardcode certain configurations. During initialization, a TreeExecutorNode parses the provided options
 * and activates/deactivates the corresponding features.
 */
class TreeExecutorNodeOptions
{
public:
  /**
   * @brief Constructor.
   *
   * Executor options must be created by passing an existing `rclcpp::NodeOptions` object.
   * @param ros_node_options ROS 2 node options.
   */
  TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options);

  /**
   * @brief Configure whether the executor node accepts scripting enum parameters.
   * @param from_overrides `true` allows to set scripting enums from parameter overrides, `false` forbids that.
   * @param dynamic `true` allows to dynamically set scripting enums at runtime, `false` forbids that.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableScriptingEnumParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Configure whether the executor node accepts global blackboard parameters.
   * @param from_overrides `true` allows to set global blackboard entries from parameter overrides, `false` forbids
   * that.
   * @param dynamic `true` allows to dynamically set global blackboard entries at runtime, `false` forbids that.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & enableGlobalBlackboardParameters(bool from_overrides, bool dynamic);

  /**
   * @brief Specify a default behavior tree build handler that will be used initially.
   * @param name Fully qualified class name of the behavior tree build handler plugin.
   * @return Modified options object.
   */
  TreeExecutorNodeOptions & setDefaultBuildHandler(const std::string & name);

  /**
   * @brief Get the ROS 2 node options that comply with the given options.
   * @return Corresponding `rclcpp::NodeOptions` object.
   */
  rclcpp::NodeOptions getROSNodeOptions() const;

private:
  friend class TreeExecutorNode;

  rclcpp::NodeOptions ros_node_options_;
  bool scripting_enum_parameters_from_overrides_ = true;
  bool scripting_enum_parameters_dynamic_ = true;
  bool blackboard_parameters_from_overrides_ = true;
  bool blackboard_parameters_dynamic_ = true;
  std::map<std::string, rclcpp::ParameterValue> custom_default_parameters_;
};

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Flexible ROS 2 node implementing a standardized interface for dynamically executing behavior trees.
 *
 * The executor is configured using ROS 2 parameters. Parameters are typically set using parameter overrides when
 * launching a node or during runtime using the `ros2 param set` command line tool.
 *
 * A behavior tree can be executed via command line:
 *
 * ```sh
 * ros2 run auto_apms_behavior_tree run_behavior <build_request>
 * ```
 *
 * Alternatively, an executor can also be included as part of a ROS 2 components container. The following executor
 * components are provided:
 *
 * - `%auto_apms_behavior_tree::TreeExecutorNode`
 *
 * - `auto_apms_behavior_tree::NoUndeclaredParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyScriptingEnumParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyBlackboardParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyInitialScriptingEnumParamsExecutorNode`
 *
 * - `auto_apms_behavior_tree::OnlyInitialBlackboardParamsExecutorNode`
 */
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
   * @brief Constructor allowing to specify a custom node name and executor options.
   * @param name Default name of the `rclcpp::Node`.
   * @param executor_options Executor specific options. Simply pass a `rclcpp::NodeOptions` object to use the default
   * options.
   */
  TreeExecutorNode(const std::string & name, TreeExecutorNodeOptions executor_options);

  /**
   * @brief Constructor populating both the node's name and the executor options with the default.
   * @param options Options forwarded to rclcpp::Node constructor.
   */
  explicit TreeExecutorNode(rclcpp::NodeOptions options);

  virtual ~TreeExecutorNode() override = default;

  using TreeExecutorBase::startExecution;

  /**
   * @brief Start the behavior tree that is specified by a particular build request.
   *
   * Executing the behavior tree is achieved by regularly invoking the internal routine that ticks the behavior tree
   * created using @p make_tree. This requires to register a timer with the associated ROS 2 node. Consequently, the
   * behavior tree is executed asynchronously. The user is provided a shared future object that allows to check whether
   * the execution finished. Once this future completes, the execution result can be evaluated.
   * @param tree_build_request Behavior tree build request forwarded to the currently loaded build handler.
   * @return Shared future that completes once executing the tree is finished or an error occurs. In that case, it is
   * assigned an execution result code.
   */
  std::shared_future<ExecutionResult> startExecution(const std::string & tree_build_request);

private:
  /* Virtual methods */

  /**
   * @brief Callback invoked every time before any behavior trees are built.
   *
   * This is invoked first thing inside the tree constructor callback returned by TreeExecutorNode::makeTreeConstructor
   * just after the tree builder object has been instantiated. Therefore, this allows the user to define an executor
   * specific initial configuration of the builder object, before the underlying tree document is passed to the
   * currently configured build handler and the tree is instantiated.
   * @param builder Tree builder to be configured. This is used for creating the behavior tree later.
   * @param node_manifest Behavior tree node manifest that specifies which nodes to use and how to load them. It is
   * provided by the `StartTreeExecutor` goal request sent to this executor.
   */
  virtual void setUpBuilder(TreeBuilder & builder, const core::NodeManifest & node_manifest);

protected:
  /* Utility methods */

  /**
   * @brief Assemble all parameters of this node that have a specific prefix.
   *
   * This function searches with depth = 2, so given that `prefix = "foo"` it will only consider parameters with names
   * that match `foo.bar` but not `foo.bar1.bar2` and deeper patterns. The user must specify @p prefix accordingly.
   * @param prefix Only consider parameters that have this prefix in their names.
   * @return Map of parameter names and their respective values.
   */
  std::map<std::string, rclcpp::ParameterValue> getParameterValuesWithPrefix(const std::string & prefix);

  /**
   * @brief Get the name of a parameter without its prefix.
   * @param prefix Prefix to remove from @p param_name.
   * @param param_name Name of the parameter with its prefix.
   * @return Name of the parameter without its prefix. If @p param_name doesn't contain @p prefix, an empty string is
   * returned.
   */
  static std::string stripPrefixFromParameterName(const std::string & prefix, const std::string & param_name);

  /**
   * @brief Update the internal buffer of scripting enums used when a behavior tree is created.
   *
   * This function automatically deduces the underlying type of each parameter value in @p value_map. Scripting enums
   * are stored as integers, so if a parameter value is not `int` or `bool` (`true == 1` and `false == 0`) an error is
   * raised. If @p simulate is `true`, the buffer won't be modified and instead of raising an error this function simply
   * returns `false`.
   * @param value_map Map of parameter names and their respective values.
   * @param simulate Set this to `true` to only validate that the underlying types of the provided parameter values
   * comply with the above mentioned requirements.
   * @return `true` if updating the scripting enums using @p value_map is possible, `false` otherwise. Only if @p
   * simulate is `false`, they are actually updated.
   * @throw auto_apms_behavior_tree::exceptions::ParameterConversionError if a parameter value cannot be converted to a
   * valid scripting enum.
   */
  bool updateScriptingEnumsWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate = false);

  /**
   * @brief Update the global blackboard using parameter values.
   *
   * This function automatically deduces the underlying type of each parameter value in @p value_map and sets the global
   * blackboard entries determined by the map's keys accordingly. If a blackboard entry already exists, it is only
   * allowed to modify it with a value that has the same type. If the types mismatch, an error is raised. If @p simulate
   * is `true`, the blackboard is not modified and instead of raising an error this function simply returns `false`.
   * @param value_map Map of parameter names and their respective values.
   * @param simulate Set this to `true` to only validate that the underlying types of the provided parameter values
   * comply with the above mentioned requirements.
   * @return `true` if updating the global blackboard using @p value_map is possible, `false` otherwise. Only if @p
   * simulate is `false`, it is actually updated.
   * @throw auto_apms_behavior_tree::exceptions::ParameterConversionError if a parameter value cannot be converted to a
   * valid blackboard entry.
   * @throw BT::LogicError if the user tries to change the type of an existing blackboard entry.
   */
  bool updateGlobalBlackboardWithParameterValues(
    const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate = false);

  /**
   * @brief Load a particular behavior tree build handler plugin.
   *
   * This enables this executor to dynamically change the build handler which accepts incoming build requests.
   * @param name Fully qualified name of the respective behavior tree build handler class. Set to `none` to simply
   * unload the current build handler.
   */
  void loadBuildHandler(const std::string & name);

  /**
   * @brief Create a callback that builds a behavior tree according to a specific request.
   *
   * The created callback makes all defined scripting enums available for the behavior tree and invokes the currently
   * configured build handler to build it. It returns a corresponding instance of `BT::Tree` that may be ticked to
   * execute the tree.
   * @param build_handler_request Request that specifies how to build the behavior tree encoded in a string.
   * @param root_tree_name Name of the requested root tree.
   * @param node_manifest Behavior tree node manifest that specifies which nodes to use and how to load them.
   * @param node_overrides Behavior tree node manifest that specifies which nodes to override once the tree has been
   * built. This may be used to swap specific node plugins that have been loaded by the build handler.
   * @return Callback for creating the behavior tree according to the build request.
   * @throw auto_apms_behavior_tree::exceptions::TreeBuildError if the build handler rejects the request.
   */
  TreeConstructor makeTreeConstructor(
    const std::string & build_handler_request, const std::string & root_tree_name,
    const core::NodeManifest & node_manifest = {}, const core::NodeManifest & node_overrides = {});

  /**
   * @brief Reset the global blackboard and clear all entries. This also unsets the corresponding parameters.
   * @return `true` if blackboard was cleared, `false` if executor is not idle meaning that the blackboard cannot be
   * cleared.
   */
  virtual bool clearGlobalBlackboard() override;

private:
  /* Executor specific virtual overrides */

  bool onTick() override final;

  bool afterTick() override final;

  void onTermination(const ExecutionResult & result) override final;

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
  ExecutorParameterListener executor_param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_ptr_;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_event_handler_ptr_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr parameter_event_callback_handle_ptr_;
  core::NodeRegistrationLoader::SharedPtr tree_node_loader_ptr_;
  TreeBuildHandlerLoader::UniquePtr build_handler_loader_ptr_;
  TreeBuilder::UniquePtr builder_ptr_;
  TreeBuildHandler::UniquePtr build_handler_ptr_;
  std::string current_build_handler_name_;
  std::map<std::string, int> scripting_enums_;
  std::map<std::string, rclcpp::ParameterValue> translated_global_blackboard_entries_;
  TreeConstructor tree_constructor_;

  // Interface objects
  rclcpp_action::Server<StartActionContext::Type>::SharedPtr start_action_ptr_;
  StartActionContext start_action_context_;
  rclcpp_action::Server<CommandActionContext::Type>::SharedPtr command_action_ptr_;
  rclcpp::TimerBase::SharedPtr command_timer_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_blackboard_service_ptr_;
};

}  // namespace auto_apms_behavior_tree