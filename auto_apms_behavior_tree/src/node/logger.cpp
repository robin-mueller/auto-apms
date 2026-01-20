// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "auto_apms_behavior_tree_core/node.hpp"
#include "rclcpp/logging.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

#define INPUT_KEY_MSG "message"
#define INPUT_KEY_LEVEL "level"

namespace auto_apms_behavior_tree
{

class Logger : public BT::SyncActionNode
{
public:
  Logger(const std::string & instance_name, const BT::NodeConfig & config, const core::RosNodeContext & context)
  : SyncActionNode(instance_name, config), context_(context)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_LEVEL, "INFO",
        "Logger level. Must be one of [UNSET, DEBUG, INFO, WARN, ERROR, FATAL] but is not case sensitive."),
      BT::InputPort<BT::AnyTypeAllowed>(INPUT_KEY_MSG, "Message to be logged via rclcpp::Logger.")};
  }

  BT::NodeStatus tick() override final
  {
    const std::string level_str = getInput<std::string>(INPUT_KEY_LEVEL).value();
    int level;
    if (
      rcutils_logging_severity_level_from_string(level_str.c_str(), rcutils_get_default_allocator(), &level) !=
      RCUTILS_RET_OK) {
      const std::string error = rcutils_get_error_string().str;
      rcutils_reset_error();
      throw exceptions::RosNodeError(
        "Cannot convert input of port '" + std::string(INPUT_KEY_LEVEL) +
        "' to a valid logging severity level: " + error);
    }
    const BT::PortsRemapping::iterator it = config().input_ports.find(INPUT_KEY_MSG);
    if (it == config().input_ports.end()) return BT::NodeStatus::SUCCESS;

    std::string msg;
    if (isBlackboardPointer(it->second)) {
      // If input port is blackboard pointer, try to cast the underlying value to a string
      if (const BT::Expected<BT::Any> expected = getInput<BT::Any>(INPUT_KEY_MSG)) {
        const BT::Any & any = expected.value();
        msg = it->second + " (Cannot convert " + BT::demangle(any.type()) + " to string)";
        if (const BT::Expected<std::string> casted = any.tryCast<std::string>()) {
          msg = casted.value().c_str();
        }
      } else {
        throw exceptions::RosNodeError(context_.getFullyQualifiedTreeNodeName(this) + " - " + expected.error());
      }
    } else {
      // Otherwise print the input directly
      msg = it->second;
    }

    msg = context_.getFullyQualifiedTreeNodeName(this, false) + " - " + msg + (msg.back() == '.' ? "" : ".");
#ifdef RCUTILS_LOG_NAMED  // Not available in jazzy
    RCUTILS_LOG_NAMED(level, context_.getBaseLogger().get_name(), msg.c_str());
#else
    RCUTILS_LOG_COND_NAMED(
      level, RCUTILS_LOG_CONDITION_EMPTY, RCUTILS_LOG_CONDITION_EMPTY, context_.getBaseLogger().get_name(),
      msg.c_str());
#endif
    return BT::NodeStatus::SUCCESS;
  }

private:
  const core::RosNodeContext context_;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::Logger)