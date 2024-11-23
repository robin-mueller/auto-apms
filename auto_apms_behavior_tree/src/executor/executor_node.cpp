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

#include "auto_apms_behavior_tree/executor/executor_node.hpp"

#include <functional>
#include <regex>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/util/bt_types.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "auto_apms_util/string.hpp"
#include "rclcpp/utilities.hpp"

namespace auto_apms_behavior_tree
{

const std::vector<std::string> EXPLICITLY_ALLOWED_PARAMETERS{
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_OTHER_BUILD_HANDLERS,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_SCRIPTING_ENUMS,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_NODE,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_EXCLUDE_PACKAGES_BUILD_HANDLER,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_TICK_RATE,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_GROOT2_PORT,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER};

const std::vector<std::string> EXPLICITLY_ALLOWED_PARAMETERS_WHILE_BUSY{
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_ALLOW_DYNAMIC_BLACKBOARD,
  _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_STATE_CHANGE_LOGGER};

TreeExecutorNodeOptions::TreeExecutorNodeOptions(const rclcpp::NodeOptions & ros_node_options)
: ros_node_options_(ros_node_options)
{
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableScriptingEnumParameters(bool from_overrides, bool dynamic)
{
  scripting_enum_parameters_from_overrides_ = from_overrides;
  scripting_enum_parameters_dynamic_ = dynamic;
  return *this;
}

TreeExecutorNodeOptions & TreeExecutorNodeOptions::enableBlackboardParameters(bool from_overrides, bool dynamic)
{
  blackboard_parameters_from_overrides_ = from_overrides;
  blackboard_parameters_dynamic_ = dynamic;
  return *this;
}

rclcpp::NodeOptions TreeExecutorNodeOptions::getROSNodeOptions()
{
  ros_node_options_.automatically_declare_parameters_from_overrides(
    scripting_enum_parameters_from_overrides_ || blackboard_parameters_from_overrides_);
  ros_node_options_.allow_undeclared_parameters(scripting_enum_parameters_dynamic_ || blackboard_parameters_dynamic_);
  return ros_node_options_;
}

TreeExecutorNode::TreeExecutorNode(const std::string & name, TreeExecutorNodeOptions executor_options)
: TreeExecutorBase(std::make_shared<rclcpp::Node>(name, executor_options.getROSNodeOptions())),
  executor_options_(executor_options),
  executor_param_listener_(node_ptr_),
  start_action_context_(logger_)
{
  const ExecutorParameters initial_params = executor_param_listener_.get_params();

  // Remove all unkown parameters provided via parameter overrides
  rcl_interfaces::msg::ListParametersResult res = node_ptr_->list_parameters({}, 0);
  std::vector<std::string> unkown_param_names;
  for (const std::string & param_name : res.names) {
    if (!stripPrefixFromParameterName(SCRIPTING_ENUM_PARAM_PREFIX, param_name).empty()) continue;
    if (!stripPrefixFromParameterName(BLACKBOARD_PARAM_PREFIX, param_name).empty()) continue;
    if (auto_apms_util::contains(EXPLICITLY_ALLOWED_PARAMETERS, param_name)) continue;
    try {
      node_ptr_->undeclare_parameter(param_name);
    } catch (const rclcpp::exceptions::ParameterImmutableException & e) {
      // Allow all builtin read only parameters
      continue;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
      // Allow all builtin statically typed parameters
      continue;
    }
    unkown_param_names.push_back(param_name);
  }
  if (!unkown_param_names.empty()) {
    RCLCPP_WARN(
      logger_, "The following initial parameters are not supported and have been removed: [ %s ].",
      rcpputils::join(unkown_param_names, ", ").c_str());
  }

  // Create behavior tree node loader
  node_loader_ptr_ = core::NodeRegistrationLoader::make_shared(
    std::set<std::string>(initial_params.node_exclude_packages.begin(), initial_params.node_exclude_packages.end()));

  // Create behavior tree build handler loader
  build_handler_loader_ptr_ = TreeBuildHandlerLoader::make_unique(std::set<std::string>(
    initial_params.build_handler_exclude_packages.begin(), initial_params.build_handler_exclude_packages.end()));

  // Instantiate behavior tree build handler
  loadBuildHandler(initial_params.build_handler);

  // Collect scripting enum and blackboard parameters from initial parameters
  const auto initial_scripting_enums = getParameterValuesWithPrefix(SCRIPTING_ENUM_PARAM_PREFIX);
  if (!initial_scripting_enums.empty()) {
    if (executor_options_.scripting_enum_parameters_from_overrides_) {
      updateScriptingEnumsWithParameterValues(initial_scripting_enums);
    } else {
      RCLCPP_WARN(
        logger_,
        "Initial scripting enums have been provided, but the 'Scripting enums from overrides' option is disabled. "
        "Ignoring.");
    }
  }
  const auto initial_blackboard = getParameterValuesWithPrefix(BLACKBOARD_PARAM_PREFIX);
  if (!initial_blackboard.empty()) {
    if (executor_options_.blackboard_parameters_from_overrides_) {
      updateBlackboardWithParameterValues(initial_blackboard, *getGlobalBlackboardPtr());
    } else {
      RCLCPP_WARN(
        logger_,
        "Initial blackboard entries have been provided, but the 'Blackboard from overrides' option is disabled. "
        "Ignoring.");
    }
  }

  using namespace std::placeholders;
  start_action_ptr_ = rclcpp_action::create_server<StartActionContext::Type>(
    node_ptr_, std::string(node_ptr_->get_name()) + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_START_ACTION_NAME_SUFFIX,
    std::bind(&TreeExecutorNode::handle_start_goal_, this, _1, _2),
    std::bind(&TreeExecutorNode::handle_start_cancel_, this, _1),
    std::bind(&TreeExecutorNode::handle_start_accept_, this, _1));

  command_action_ptr_ = rclcpp_action::create_server<CommandActionContext::Type>(
    node_ptr_, std::string(node_ptr_->get_name()) + _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_COMMAND_ACTION_NAME_SUFFIX,
    std::bind(&TreeExecutorNode::handle_command_goal_, this, _1, _2),
    std::bind(&TreeExecutorNode::handle_command_cancel_, this, _1),
    std::bind(&TreeExecutorNode::handle_command_accept_, this, _1));

  // Adding the local on_set_parameters_callback after the parameter listeners from generate_parameters_library
  // are created makes sure that this callback will be evaluated before the listener callbacks.
  // This is desired to keep the internal parameter struct in sync, because the callbacks of the listeners implicitly
  // set them if the change is accepted. Otherwise, they would be set even if the local callback rejects the change.
  // We DO NOT set any variables in this callback, but only check if the request to change certain parameters is valid.
  // The actual change is performed in the callback registered with rclcpp::ParameterEventListener
  on_set_parameters_callback_handle_ptr_ =
    node_ptr_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->on_set_parameters_callback_(parameters);
    });

  // Add an event handler that applies the actual parameter change
  parameter_event_handler_ptr_ = std::make_shared<rclcpp::ParameterEventHandler>(node_ptr_);
  parameter_event_callback_handle_ptr_ = parameter_event_handler_ptr_->add_parameter_event_callback(
    [this](const rcl_interfaces::msg::ParameterEvent & event) { this->parameter_event_callback_(event); });

  // Make sure ROS arguments are removed. When applying composition, this is typically not the case.
  std::vector<std::string> args_with_ros_arguments = executor_options.getROSNodeOptions().arguments();
  int argc = args_with_ros_arguments.size();
  char ** argv = new char *[argc + 1];  // +1 for the null terminator
  for (int i = 0; i < argc; ++i) {
    argv[i] = const_cast<char *>(args_with_ros_arguments[i].c_str());
  }
  argv[argc] = nullptr;  // Null-terminate the array as required for argv[]

  // Evaluate possible cli argument dictating to start executing with a specific build request immediately.
  // Note: First argument is always path of executable.
  if (const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv); args.size() > 1) {
    // Log relevant arguments. First argument is executable name (argv[0]) and won't be considered.
    std::vector<std::string> relevant_args{args.begin() + 1, args.end()};
    RCLCPP_DEBUG(
      logger_, "Additional cli arguments in rclcpp::NodeOptions: [ %s ]", rcpputils::join(relevant_args, ", ").c_str());

    // Start tree execution with the build handler request being the first relevant argument
    startExecution(makeTreeConstructor(relevant_args[0], ""), initial_params.tick_rate, initial_params.groot2_port);
  }
}

TreeExecutorNode::TreeExecutorNode(rclcpp::NodeOptions options)
: TreeExecutorNode(_AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options))
{
}

void TreeExecutorNode::setUpBuilder(core::TreeBuilder & /*builder*/) {}

std::map<std::string, rclcpp::ParameterValue> TreeExecutorNode::getParameterValuesWithPrefix(const std::string & prefix)
{
  // Get all parameters with prefix <prefix> and separator depth 2 (Will match all names like <prefix>.<suffix> but not
  // <prefix>.<suffix1>.<suffix2> and deeper)
  const auto res = node_ptr_->list_parameters({prefix}, 2);
  std::map<std::string, rclcpp::ParameterValue> value_map;
  for (const std::string & name_with_prefix : res.names) {
    if (const std::string suffix = stripPrefixFromParameterName(prefix, name_with_prefix); !suffix.empty()) {
      value_map[suffix] = node_ptr_->get_parameter(name_with_prefix).get_parameter_value();
    }
  }
  return value_map;
}

std::string TreeExecutorNode::stripPrefixFromParameterName(const std::string & prefix, const std::string & param_name)
{
  const std::regex reg("^" + prefix + "\\.(\\S+)");
  if (std::smatch match; std::regex_match(param_name, match, reg)) return match[1].str();
  return "";
}

bool TreeExecutorNode::updateScriptingEnumsWithParameterValues(
  const std::map<std::string, rclcpp::ParameterValue> & value_map, bool simulate)
{
  std::map<std::string, std::string> set_successfully_map;
  for (const auto & [enum_key, pval] : value_map) {
    try {
      switch (pval.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (!simulate) scripting_enums_[enum_key] = static_cast<int>(pval.get<bool>());
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (!simulate) scripting_enums_[enum_key] = pval.get<int64_t>();
          break;
        default:
          throw std::runtime_error("Parameter to scripting enum conversion is not defined.");
      }
      set_successfully_map[enum_key] = rclcpp::to_string(pval);
    } catch (const std::exception & e) {
      if (!simulate) {
        RCLCPP_ERROR(
          logger_, "Error setting scripting enum from parameter %s=%s (Type: %s): %s", enum_key.c_str(),
          rclcpp::to_string(pval).c_str(), rclcpp::to_string(pval.get_type()).c_str(), e.what());
      }
      return false;
    }
  }
  if (!simulate) {
    RCLCPP_DEBUG(
      logger_, "Updated scripting enums from parameters: { %s }",
      auto_apms_util::printMap(set_successfully_map).c_str());
  }
  return true;
}

bool TreeExecutorNode::updateBlackboardWithParameterValues(
  const std::map<std::string, rclcpp::ParameterValue> & value_map, TreeBlackboard & bb, bool simulate)
{
  std::map<std::string, std::string> set_successfully_map;
  for (const auto & [entry_key, pval] : value_map) {
    try {
      switch (pval.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<bool>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<bool>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<int64_t>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<int64_t>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<double>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<double>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::string>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::string>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::vector<uint8_t>>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::vector<uint8_t>>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::vector<bool>>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::vector<bool>>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::vector<int64_t>>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::vector<int64_t>>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::vector<double>>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::vector<double>>());
          }
          break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          if (simulate) {
            if (std::shared_ptr<TreeBlackboard::Entry> entry_ptr = bb.getEntry(entry_key);
                entry_ptr && !entry_ptr->value.isType<std::vector<std::string>>()) {
              return false;
            }
          } else {
            bb.set(entry_key, pval.get<std::vector<std::string>>());
          }
          break;
        default:
          throw std::runtime_error("Parameter to blackboard conversion is not defined.");
      }
      set_successfully_map[entry_key] = rclcpp::to_string(pval);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Error updating blackboard from parameter %s=%s (Type: %s): %s", entry_key.c_str(),
        rclcpp::to_string(pval).c_str(), rclcpp::to_string(pval.get_type()).c_str(), e.what());
      return false;
    }
  }
  if (!simulate) {
    RCLCPP_DEBUG(
      logger_, "Updated blackboard from parameters: { %s }", auto_apms_util::printMap(set_successfully_map).c_str());
  }
  return true;
}

void TreeExecutorNode::loadBuildHandler(const std::string & name)
{
  if (build_handler_ptr_ && !executor_param_listener_.get_params().allow_other_build_handlers) {
    throw std::logic_error(
      "Executor option 'Allow other build handlers' is disabled, but loadBuildHandler() was called again after "
      "instantiating '" +
      current_build_handler_name_ + "'.");
  }
  if (current_build_handler_name_ == name) return;
  if (name == PARAM_VALUE_NO_BUILD_HANDLER) {
    build_handler_ptr_.reset();
  } else {
    try {
      build_handler_ptr_ = build_handler_loader_ptr_->createUniqueInstance(name)->makeUnique(node_ptr_);
    } catch (const std::exception & e) {
      throw exceptions::TreeBuildError(
        "An error occurred when trying to create an instance of tree build handler class '" + name +
        "'. Remember that the AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER macro must be "
        "called in the source file for the class to be discoverable. Error message: " +
        e.what());
    }
  }
  current_build_handler_name_ = name;
}

TreeConstructor TreeExecutorNode::makeTreeConstructor(
  const std::string & build_handler_request, const std::string & root_tree_name,
  const core::NodeManifest & node_overrides)
{
  // Request the tree identity
  if (build_handler_ptr_ && !build_handler_ptr_->setBuildRequest(build_handler_request, root_tree_name)) {
    throw exceptions::TreeBuildError(
      "Build request '" + build_handler_request + "' was denied by '" +
      executor_param_listener_.get_params().build_handler + "' (setBuildRequest() returned false).");
  }

  // By passing the the local variables to the callback's captures by value they live on and can be used for creating
  // the tree later. Otherwise a segmentation fault might occur since memory allocated for the arguments might be
  // released at the time the method returns.
  return [this, root_tree_name, node_overrides](TreeBlackboardSharedPtr bb_ptr) {
    TreeBuilder builder(node_ptr_, node_loader_ptr_);

    // Allow executor to set up the builder independently from the build handler
    setUpBuilder(builder);

    // Make scripting enums available to tree instance
    for (const auto & [enum_key, val] : scripting_enums_) builder.setScriptingEnum(enum_key, val);

    // If a build handler is specified, let it configure the builder and determine which tree is to be instantiated
    std::string instantiate_name = root_tree_name;
    if (build_handler_ptr_) {
      instantiate_name = build_handler_ptr_->buildTree(builder, *bb_ptr).getName();
    }

    // Allow for overriding selected node instances
    builder.loadNodePlugins(node_overrides, true);

    // Finally, instantiate the tree
    return builder.instantiate(instantiate_name, bb_ptr);
  };
}

rcl_interfaces::msg::SetParametersResult TreeExecutorNode::on_set_parameters_callback_(
  const std::vector<rclcpp::Parameter> & parameters)
{
  const ExecutorParameters params = executor_param_listener_.get_params();

  // Iterate through parameters and individually decide wether to reject the change
  for (const auto & p : parameters) {
    const std::string param_name = p.get_name();
    auto create_rejected = [&param_name](const std::string msg) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Rejected to set parameter '" + param_name + "': " + msg + ".";
      return result;
    };

    // Check if parameter is a scripting enum
    if (const std::string enum_key = stripPrefixFromParameterName(SCRIPTING_ENUM_PARAM_PREFIX, param_name);
        !enum_key.empty()) {
      if (isBusy()) {
        return create_rejected("Scripting enums cannot change while tree executor is running");
      }
      if (!executor_options_.scripting_enum_parameters_dynamic_ || !params.allow_dynamic_scripting_enums) {
        return create_rejected(
          "Cannot set scripting enum '" + enum_key + "', because the 'Dynamic scripting enums' option is disabled");
      }
      // Validate type of scripting enum parameters
      if (!updateScriptingEnumsWithParameterValues({{enum_key, p.get_parameter_value()}}, true)) {
        return create_rejected(
          "Type of scripting enum must be bool or int. Tried to set enum '" + enum_key + "' with value '" +
          p.value_to_string() + "' (Type: " + p.get_type_name() + ")");
      };
      // If scripting enum is allowed to change, continue with next parameter.
      continue;
    }

    // Check if parameter is a blackboard parameter
    if (const std::string entry_key = stripPrefixFromParameterName(BLACKBOARD_PARAM_PREFIX, param_name);
        !entry_key.empty()) {
      if (!executor_options_.blackboard_parameters_dynamic_ || !params.allow_dynamic_blackboard) {
        return create_rejected(
          "Cannot set blackboard entry '" + entry_key + "', because the 'Dynamic blackboard' option is disabled");
      }
      // Validate type of blackboard parameters won't change
      if (!updateBlackboardWithParameterValues(
            {{entry_key, p.get_parameter_value()}}, *getGlobalBlackboardPtr(), true)) {
        return create_rejected(
          "Type of blackboard entries must not change. Tried to set entry '" + entry_key +
          "' (Type: " + getGlobalBlackboardPtr()->getEntry(entry_key)->info.typeName() + ") with value '" +
          p.value_to_string() + "' (Type: " + p.get_type_name() + ")");
      };
      // If blackboard entry is allowed to change, continue with next parameter.
      continue;
    }

    // Check for other parameters which are not allowed during execution
    if (isBusy() && !auto_apms_util::contains(EXPLICITLY_ALLOWED_PARAMETERS_WHILE_BUSY, param_name)) {
      return create_rejected("Parameter is not allowed to change while tree executor is running");
    }

    // Check if build handler is allowed to change and valid
    if (param_name == _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER) {
      if (!params.allow_other_build_handlers) {
        return create_rejected(
          "This executor operates with tree build handler '" + executor_param_listener_.get_params().build_handler +
          "' and doesn't allow other build handlers to be loaded since the 'Allow other build handlers' option is "
          "disabled.");
      }
      const std::string class_name = p.as_string();
      if (class_name != PARAM_VALUE_NO_BUILD_HANDLER && !build_handler_loader_ptr_->isClassAvailable(class_name)) {
        return create_rejected(
          "There is no tree build handler class named '" + class_name +
          "'. Make sure that it's spelled correctly and registered by calling "
          "auto_apms_behavior_tree_declare_build_handlers() in the CMakeLists.txt of the "
          "corresponding package");
      }
    }

    // At this point, if the parameter hasn't been declared, we do not support it.
    if (!node_ptr_->has_parameter(param_name)) {
      return create_rejected("Not one of the supported parameter names");
    }
  }

  // If not returned yet, accept to set the parameter
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void TreeExecutorNode::parameter_event_callback_(const rcl_interfaces::msg::ParameterEvent & event)
{
  // Look for any updates to parameters of this node
  std::regex re(node_ptr_->get_fully_qualified_name());
  if (std::regex_match(event.node, re)) {
    // Enumerate all changes that came in on this event
    for (const rclcpp::Parameter & p : rclcpp::ParameterEventHandler::get_parameters_from_event(event)) {
      const std::string param_name = p.get_name();

      // Change scripting enums
      if (const std::string enum_key = stripPrefixFromParameterName(SCRIPTING_ENUM_PARAM_PREFIX, param_name);
          !enum_key.empty()) {
        updateScriptingEnumsWithParameterValues({{enum_key, p.get_parameter_value()}});
      }

      // Change blackboard parameters
      if (const std::string entry_key = stripPrefixFromParameterName(BLACKBOARD_PARAM_PREFIX, param_name);
          !entry_key.empty()) {
        updateBlackboardWithParameterValues({{entry_key, p.get_parameter_value()}}, *getGlobalBlackboardPtr());
      }

      // Change tree build handler instance
      if (param_name == _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_PARAM_BUILD_HANDLER) {
        loadBuildHandler(p.as_string());
      }
    }
  }
}

rclcpp_action::GoalResponse TreeExecutorNode::handle_start_goal_(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartActionContext::Goal> goal_ptr)
{
  // Reject if a tree is already executing
  if (isBusy()) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Tree '%s' is currently executing.", rclcpp_action::to_string(uuid).c_str(),
      getTreeName().c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!goal_ptr->build_handler.empty()) {
    if (executor_param_listener_.get_params().allow_other_build_handlers) {
      try {
        loadBuildHandler(goal_ptr->build_handler);
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          logger_, "Goal %s was REJECTED: Loading tree build handler '%s' failed: %s",
          rclcpp_action::to_string(uuid).c_str(), goal_ptr->build_handler.c_str(), e.what());
        return rclcpp_action::GoalResponse::REJECT;
      }
    } else if (goal_ptr->build_handler != current_build_handler_name_) {
      RCLCPP_WARN(
        logger_,
        "Goal %s was REJECTED: Current tree build handler '%s' must not change since the 'Allow other build handlers' "
        "option is disabled.",
        rclcpp_action::to_string(uuid).c_str(), current_build_handler_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  core::NodeManifest node_overrides;
  try {
    node_overrides = core::NodeManifest::decode(goal_ptr->node_overrides);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Parsing the node override manifest failed: %s",
      rclcpp_action::to_string(uuid).c_str(), e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }

  try {
    tree_constructor_ = makeTreeConstructor(goal_ptr->build_request, goal_ptr->root_tree, node_overrides);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger_, "Goal %s was REJECTED: Error during makeTreeConstructor(): %s", rclcpp_action::to_string(uuid).c_str(),
      e.what());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeExecutorNode::handle_start_cancel_(
  std::shared_ptr<StartActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  setControlCommand(ControlCommand::TERMINATE);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutorNode::handle_start_accept_(std::shared_ptr<StartActionContext::GoalHandle> goal_handle_ptr)
{
  // Clear blackboard parameters if desired
  if (goal_handle_ptr->get_goal()->clear_blackboard) {
    const auto res = node_ptr_->list_parameters({BLACKBOARD_PARAM_PREFIX}, 2);
    for (const std::string & name : res.names) {
      node_ptr_->undeclare_parameter(name);
    };
    clearGlobalBlackboard();
  }

  const ExecutorParameters params = executor_param_listener_.get_params();
  try {
    startExecution(tree_constructor_, params.tick_rate, params.groot2_port);
  } catch (const std::exception & e) {
    auto result_ptr = std::make_shared<StartActionContext::Result>();
    result_ptr->message = "An error occurred trying to start execution: " + std::string(e.what());
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

rclcpp_action::GoalResponse TreeExecutorNode::handle_command_goal_(
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

rclcpp_action::CancelResponse TreeExecutorNode::handle_command_cancel_(
  std::shared_ptr<CommandActionContext::GoalHandle> /*goal_handle_ptr*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutorNode::handle_command_accept_(std::shared_ptr<CommandActionContext::GoalHandle> goal_handle_ptr)
{
  const auto command_request = goal_handle_ptr->get_goal()->command;
  ExecutionState requested_state;
  switch (command_request) {
    case CommandActionContext::Goal::COMMAND_RESUME:
      setControlCommand(ControlCommand::RUN);
      requested_state = ExecutionState::RUNNING;
      break;
    case CommandActionContext::Goal::COMMAND_PAUSE:
      setControlCommand(ControlCommand::PAUSE);
      requested_state = ExecutionState::PAUSED;
      break;
    case CommandActionContext::Goal::COMMAND_HALT:
      setControlCommand(ControlCommand::HALT);
      requested_state = ExecutionState::HALTED;
      break;
    case CommandActionContext::Goal::COMMAND_TERMINATE:
      setControlCommand(ControlCommand::TERMINATE);
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
          logger_, "Failed to reach requested state %s due to cancellation of execution timer. Aborting.",
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

bool TreeExecutorNode::onTick()
{
  const ExecutorParameters params = executor_param_listener_.get_params();
  auto & state_observer = getStateObserver();

  /**
   * Update entities using dynamic parameters
   */
  state_observer.setLogging(params.state_change_logger);

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

void TreeExecutorNode::onTermination(const ExecutionResult & result)
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
      result_ptr->message = "An unexpected error occurred during tree execution";
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