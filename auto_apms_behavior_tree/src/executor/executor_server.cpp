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
#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_util/string.hpp"

namespace auto_apms_behavior_tree
{

// clang-format off
const std::string TreeExecutorServer::PARAM_NAME_TREE_BUILDER = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TREE_CREATOR;
const std::string TreeExecutorServer::PARAM_NAME_TICK_RATE = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE;
const std::string TreeExecutorServer::PARAM_NAME_GROOT2_PORT = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT;
const std::string TreeExecutorServer::PARAM_NAME_STATE_CHANGE_LOGGER = _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER;
// clang-format on

TreeExecutorServer::TreeExecutorServer(const std::string & name, rclcpp::NodeOptions options)
: TreeExecutor(std::make_shared<rclcpp::Node>(
    name, options.automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true))),
  logger_(node_ptr_->get_logger()),
  executor_param_listener_(node_ptr_),
  start_action_context_(logger_),
  command_action_context_(logger_)
{
  using namespace std::placeholders;
  start_action_ptr_ = rclcpp_action::create_server<StartActionContext::Type>(
    node_ptr_, name + BT_EXECUTOR_RUN_ACTION_NAME_SUFFIX,
    std::bind(&TreeExecutorServer::handle_start_goal_, this, _1, _2),
    std::bind(&TreeExecutorServer::handle_start_cancel_, this, _1),
    std::bind(&TreeExecutorServer::handle_start_accept_, this, _1));

  command_action_ptr_ = rclcpp_action::create_server<CommandActionContext::Type>(
    node_ptr_, name + BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX,
    std::bind(&TreeExecutorServer::handle_command_goal_, this, _1, _2),
    std::bind(&TreeExecutorServer::handle_command_cancel_, this, _1),
    std::bind(&TreeExecutorServer::handle_command_accept_, this, _1));

  // Adding the local on_set_parameters_callback after the parameter listeners from generate_parameters_library
  // are created makes sure that this callback will be evaluated before the listener callbacks.
  // This is desired to keep the internal parameter struct in sync, because the callbacks of the listeners implicitly
  // set them if the change is accepted. Otherwise, they would be set even if the local callback rejects the change.
  on_set_parameters_callback_handle_ =
    node_ptr_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->on_set_parameters_callback_(parameters);
    });

  // Evaluate possible cli argument dictating to start executing a specific tree immediately
  auto & args = options.arguments();
  if (args.size() > 1)  // First argument is always path of executable
  {
    auto params = executor_param_listener_.get_params();
    std::vector<std::string> additional_args{args.begin() + 1, args.end()};
    RCLCPP_DEBUG(
      logger_, "Additional cli arguments in rclcpp::NodeOptions: %s", rcpputils::join(additional_args, ", ").c_str());
    startExecution(makeCreateTreeCallback(params.tree_creator_name, args[1]), params.tick_rate, params.groot2_port);
  }
}

TreeExecutorServer::TreeExecutorServer(const rclcpp::NodeOptions & options)
: TreeExecutorServer(DEFAULT_NODE_NAME, options)
{
}

void TreeExecutorServer::prepareTreeBuilder(TreeBuilder & /*builder*/) {}

bool TreeExecutorServer::isDynamicBlackboardParamName(const std::string & name)
{
  return name.find(DYNAMIC_BB_PARAM_PREFIX + DYNAMIC_BB_PARAM_SEPARATOR) == 0;
}

void TreeExecutorServer::configureBlackboardFromParameters(TreeBlackboard & bb)
{
  // Get all parameter entries with prefix bb and depth 2 (Will match all names like bb.foo but not bb.foo.bar)
  const auto res = node_ptr_->list_parameters({DYNAMIC_BB_PARAM_PREFIX}, 2);
  std::map<std::string, std::string> entries;
  for (const std::string & name_with_prefix : res.names) {
    if (const std::size_t dot_pos = name_with_prefix.find(DYNAMIC_BB_PARAM_SEPARATOR);
        dot_pos != std::string::npos && dot_pos + 1 < name_with_prefix.length()) {
      const std::string name = name_with_prefix.substr(dot_pos + 1);
      entries[name] = node_ptr_->get_parameter(name_with_prefix).value_to_string();
    }
  }
  if (dynamic_bb_param_entries_ != entries) {
    for (const auto & [name, val] : entries) {
      try {
        bb.set(name, val);  // Use global blackboard idiom
      } catch (const std::exception & e) {
        RCLCPP_ERROR(logger_, "Error setting blackboard entry %s=%s: %s", name.c_str(), val.c_str(), e.what());
        return;
      }
    }
    dynamic_bb_param_entries_ = entries;
    RCLCPP_DEBUG(
      logger_, "Set global blackboard entries from parameters: { %s }",
      auto_apms_util::printMap(dynamic_bb_param_entries_).c_str());
  }
}

TreeExecutorServer::CreateTreeCallback TreeExecutorServer::makeCreateTreeCallback(
  const std::string & tree_creator_name, const std::string & tree_creator_request, const NodeManifest & node_overrides)
{
  // Make sure builder is available
  if (!tree_creator_loader_.isClassAvailable(tree_creator_name)) {
    throw exceptions::TreeBuildError(
      "There is no tree builder class named '" + tree_creator_name +
      "'. Make sure that it's spelled correctly and registered by calling "
      "auto_apms_behavior_tree_register_creators() in the CMakeLists.txt of the "
      "corresponding package.");
  }

  // Try to load and create the tree builder
  std::shared_ptr<TreeCreatorBase> creator_ptr;
  try {
    creator_ptr = tree_creator_loader_.createUniqueInstance(tree_creator_name)->instantiateCreator(node_ptr_);
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError(
      "An error occured when trying to create an instance of tree builder '" + tree_creator_name +
      "'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_CREATOR macro must be "
      "called in the source file for the builder class to be discoverable. Error "
      "message: " +
      e.what());
  }

  // Request the tree identity
  if (!creator_ptr->setRequest(tree_creator_request)) {
    throw exceptions::TreeBuildError(
      "Request to build tree '" + tree_creator_request + "' was denied by tree builder '" + tree_creator_name +
      "'(setRequest() returned false).");
  }

  // By passing the creator's shared pointer to the callback, it lives on and the tree can be built later.
  return [this, node_overrides, creator_ptr](TreeBlackboardSharedPtr bb_ptr) {
    // Set up blackboard using dynamic parameters
    configureBlackboardFromParameters(*bb_ptr);

    // Run build pipeline
    TreeBuilder builder(node_ptr_, node_loader_ptr_);
    prepareTreeBuilder(builder);
    creator_ptr->configureTreeBuilder(builder);
    creator_ptr->configureBlackboard(*bb_ptr);
    builder.loadNodePlugins(node_overrides, true);
    return creator_ptr->createTree(builder, bb_ptr);
  };
}

rcl_interfaces::msg::SetParametersResult TreeExecutorServer::on_set_parameters_callback_(
  const std::vector<rclcpp::Parameter> & parameters)
{
  // Parameters allowed to be set while busy
  const std::set<std::string> allowed_while_busy{PARAM_NAME_STATE_CHANGE_LOGGER};

  // Iterate through parameters and individually decide wether to reject the change
  for (const auto & p : parameters) {
    const std::string param_name = p.get_name();
    auto not_in_list = [&param_name](const std::set<std::string> & list) {
      return list.find(param_name) == list.end();
    };
    auto create_rejected = [&param_name](const std::string msg) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Rejected to set parameter '" + param_name + "': " + msg;
      return result;
    };

    // Check for allowed while busy (We allow all dynamic blackboard parameters by default)
    if (isBusy() && !isDynamicBlackboardParamName(param_name) && not_in_list(allowed_while_busy)) {
      return create_rejected("Parameter cannot change when executor is busy");
    }

    // Check if builder plugin name is valid
    if (param_name == PARAM_NAME_TREE_BUILDER) {
      const std::string class_name = p.as_string();
      if (!tree_creator_loader_.isClassAvailable(class_name)) {
        return create_rejected(
          "There is no tree builder class named '" + class_name +
          "'. Make sure it is registered using the CMake macro "
          "auto_apms_behavior_tree_register_creators().");
      }
    }
  }

  // If not returned yet, accept to set the parameter
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

rclcpp_action::GoalResponse TreeExecutorServer::handle_start_goal_(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartActionContext::Goal> goal_ptr)
{
  // Reject if a tree is already executing
  if (isBusy()) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Tree '%s' is currently executing.", rclcpp_action::to_string(uuid).c_str(),
      getTreeName().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  NodeManifest node_overrides;
  try {
    node_overrides = NodeManifest::decode(goal_ptr->node_overrides);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Parsing the node override manifest failed: %s",
      rclcpp_action::to_string(uuid).c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  try {
    // Use the goal's information for determining the builder plugin to load if the string is non-empty.
    // Otherwise use the current value of the parameter.
    const std::string builder_name = goal_ptr->tree_creator_name.empty()
                                       ? executor_param_listener_.get_params().tree_creator_name
                                       : goal_ptr->tree_creator_name;
    create_tree_callback_ = makeCreateTreeCallback(builder_name, goal_ptr->tree_creator_request, node_overrides);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Error during configuring the tree builder: %s",
      rclcpp_action::to_string(uuid).c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeExecutorServer::handle_start_cancel_(
  std::shared_ptr<StartActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  setControlCommand(ControlCommand::TERMINATE);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutorServer::handle_start_accept_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr)
{
  const auto params = executor_param_listener_.get_params();
  try {
    startExecution(create_tree_callback_, params.tick_rate, params.groot2_port);
  } catch (const std::exception & e) {
    auto result_ptr = std::make_shared<StartActionContext::Result>();
    result_ptr->message = "An error occured trying to start execution: " + std::string(e.what());
    result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_NOT_SET;
    goal_handle_ptr->abort(result_ptr);
    RCLCPP_ERROR_STREAM(logger_, result_ptr->message);
    return;
  }
  const std::string started_tree_name = getTreeName();

  // If attach is true, the goal's life time is synchronized with the execution. Otherwise we succeed immediately and
  // leave the executor running (Detached mode).
  if (goal_handle_ptr->get_goal()->attach) {
    start_action_context_.setUp(goal_handle_ptr);
    RCLCPP_INFO(logger_, "Successfully started execution of tree '%s' (Mode: Attached).", started_tree_name.c_str());
  } else {
    auto result_ptr = std::make_shared<StartActionContext::Result>();
    result_ptr->message = "Successfully started execution of tree '" + started_tree_name + "' (Mode: Detached).";
    result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_NOT_SET;
    result_ptr->terminated_tree_identity = started_tree_name;
    goal_handle_ptr->succeed(result_ptr);
    RCLCPP_INFO_STREAM(logger_, result_ptr->message);
  }
}

rclcpp_action::GoalResponse TreeExecutorServer::handle_command_goal_(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const CommandActionContext::Goal> goal_ptr)
{
  if (command_timer_ptr_ && !command_timer_ptr_->is_canceled()) {
    RCLCPP_WARN(logger_, "Request for setting tree executor command rejected, because previous one is still busy.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (isBusy() && start_action_context_.isValid() && start_action_context_.getGoalHandlePtr()->is_canceling()) {
    RCLCPP_WARN(logger_, "Request for setting tree executor command rejected, because tree executor is canceling.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto execution_state = getExecutionState();
  switch (goal_ptr->command) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      if (execution_state == ExecutionState::PAUSED || execution_state == ExecutionState::HALTED) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will RESUME.", getTreeName().c_str());
      } else {
        RCLCPP_WARN(
          logger_, "Requested to RESUME with executor being in state %s. Rejecting request.",
          toStr(execution_state).c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      if (execution_state == ExecutionState::STARTING || execution_state == ExecutionState::RUNNING) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will PAUSE", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to PAUSE with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      if (
        execution_state == ExecutionState::STARTING || execution_state == ExecutionState::RUNNING ||
        execution_state == ExecutionState::PAUSED) {
        RCLCPP_INFO(logger_, "Tree with ID '%s' will HALT.", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to HALT with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      if (isBusy()) {
        RCLCPP_INFO(logger_, "Executor will TERMINATE tree '%s'.", getTreeName().c_str());
      } else {
        RCLCPP_INFO(
          logger_, "Requested to TERMINATE with executor already being inactive (State: %s).",
          toStr(execution_state).c_str());
      }
      break;
    default:
      RCLCPP_WARN(logger_, "Executor command %i is undefined. Rejecting request.", goal_ptr->command);
      return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeExecutorServer::handle_command_cancel_(
  std::shared_ptr<CommandActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutorServer::handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr)
{
  const auto command_request = goal_handle_ptr->get_goal()->command;
  switch (command_request) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      setControlCommand(ControlCommand::RUN);
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      setControlCommand(ControlCommand::PAUSE);
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      setControlCommand(ControlCommand::HALT);
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      setControlCommand(ControlCommand::TERMINATE);
      break;
  }
  ExecutionState requested_state;
  switch (command_request) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      requested_state = ExecutionState::RUNNING;
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      requested_state = ExecutionState::PAUSED;
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      requested_state = ExecutionState::HALTED;
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      requested_state = ExecutionState::IDLE;
      break;
    default:
      throw std::logic_error("command_request is unkown");
  }

  command_timer_ptr_ = node_ptr_->create_wall_timer(
    std::chrono::duration<double>(executor_param_listener_.get_params().tick_rate),
    [this, requested_state, goal_handle_ptr, action_result_ptr = std::make_shared<CommandActionContext::Result>()]() {
      // Check if canceling
      if (goal_handle_ptr->is_canceling()) {
        // Will abandon any progress
        goal_handle_ptr->canceled(action_result_ptr);
        command_timer_ptr_->cancel();
        return;
      }

      const auto current_state = getExecutionState();

      // If the execution state has become IDLE in the mean time, request failed if termination was not desired
      if (requested_state != ExecutionState::IDLE && current_state == ExecutionState::IDLE) {
        RCLCPP_ERROR(
          logger_, "Failed to reach requested state %s due to cancelation of executon timer. Aborting.",
          toStr(requested_state).c_str());
        goal_handle_ptr->abort(action_result_ptr);
        command_timer_ptr_->cancel();
        return;
      }

      // Wait for the requested state to be reached
      if (current_state != requested_state) return;

      goal_handle_ptr->succeed(action_result_ptr);
      command_timer_ptr_->cancel();
    });
}

bool TreeExecutorServer::onTick()
{
  const auto params = executor_param_listener_.get_params();
  auto & state_observer = getStateObserver();

  /**
   * Update entities using dynamic parameters
   */
  state_observer.setLogging(params.state_change_logger);
  configureBlackboardFromParameters(*getGlobalBlackboardPtr());

  // Don't send feedback if started in detached mode
  if (!start_action_context_.isValid()) return true;

  /**
   * Send feedback
   */
  auto feedback_ptr = start_action_context_.getFeedbackPtr();  // feedback from previous tick persists
  feedback_ptr->execution_state_str = toStr(getExecutionState());
  feedback_ptr->running_tree_identity = getTreeName();
  auto running_action_history = state_observer.getRunningActionHistory();
  if (!running_action_history.empty()) {
    // If there are multiple nodes running (ParallelNode), join the IDs to a single string
    feedback_ptr->running_action_name = rcpputils::join(running_action_history, " + ");
    feedback_ptr->running_action_timestamp =
      std::chrono::duration<double>{std::chrono::high_resolution_clock::now().time_since_epoch()}.count();

    // Reset the history cache
    state_observer.flush();
  }
  start_action_context_.publishFeedback();
  return true;
}

void TreeExecutorServer::onTermination(const ExecutionResult & result)
{
  if (!start_action_context_.isValid())  // Do nothing if started in detached mode
    return;

  auto result_ptr = start_action_context_.getResultPtr();
  result_ptr->terminated_tree_identity = getTreeName();
  switch (result) {
    case ExecutionResult::TREE_SUCCEEDED:
      result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_SUCCESS;
      result_ptr->message = "Tree execution finished with status SUCCESS";
      start_action_context_.succeed();
      break;
    case ExecutionResult::TREE_FAILED:
      result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_FAILURE;
      result_ptr->message = "Tree execution finished with status FAILURE";
      start_action_context_.abort();
      break;
    case ExecutionResult::TERMINATED_PREMATURELY:
      result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_NOT_SET;
      if (start_action_context_.getGoalHandlePtr()->is_canceling()) {
        result_ptr->message = "Tree execution canceled successfully";
        start_action_context_.cancel();
      } else {
        result_ptr->message = "Tree execution terminated prematurely";
        start_action_context_.abort();
      }
      break;
    case ExecutionResult::ERROR:
      result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_NOT_SET;
      result_ptr->message = "An unexpected error occured during tree execution";
      start_action_context_.abort();
      break;
    default:
      result_ptr->tree_result = StartActionContext::Result::TREE_RESULT_NOT_SET;
      result_ptr->message = "Execution result unkown";
      start_action_context_.abort();
      break;
  }

  // Reset action context
  start_action_context_.invalidate();
}

}  // namespace auto_apms_behavior_tree

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::TreeExecutorServer)