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

#include <chrono>
#include <map>
#include <string>

#include "auto_apms_util/exceptions.hpp"
#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree::core
{

/**
 * @brief Parameters for loading and registering a behavior tree node class from a shared library using
 * e.g. NodeRegistrationLoader.
 */
struct NodeRegistrationOptions
{
  static const std::string PARAM_NAME_CLASS;
  static const std::string PARAM_NAME_ROS2TOPIC;
  static const std::string PARAM_NAME_DESCRIPTION;
  static const std::string PARAM_NAME_DEFAULTS;
  static const std::string PARAM_NAME_WAIT_TIMEOUT;
  static const std::string PARAM_NAME_REQUEST_TIMEOUT;
  static const std::string PARAM_NAME_ALLOW_UNREACHABLE;
  static const std::string PARAM_NAME_LOGGER_LEVEL;
  static const std::string PARAM_NAME_EXTRA;

  /**
   * @brief Create the default node registration options.
   */
  NodeRegistrationOptions() = default;

  AUTO_APMS_UTIL_DEFINE_YAML_CONVERSION_METHODS(NodeRegistrationOptions)

  /// Fully qualified name of the behavior tree node plugin class.
  std::string class_name;
  /// Short description of the behavior tree node's purpose and use-case.
  std::string description = "No description provided.";
  /**
   * @brief Name of the ROS 2 communication interface to connect with.
   *
   * This has different meaning based on the context:
   *
   * - RosActionNode: Name of the action server
   *
   * - RosServiceNode: Name of the service
   *
   * - RosPublisherNode: Name of the topic to publish to
   *
   * - RosSubscriberNode: Name of the topic to subscribe to
   *
   * It is possible to use a port's value to define this parameter at runtime by using the special pattern
   * `(input:<port_name>)` and replacing `<port_name>` with the desired input port name.
   *
   * **Example**: Given the user implements an input port `BT::InputPort<std::string>("my_port")`, one may create a
   * client for the action "foo/bar" by defining NodeRegistrationOptions::port as `(input:my_port)/bar` and providing
   * the string "foo" to the port with name `my_port`.
   *
   * By default, we look for the communication port name using the node's input port named `port`.
   */
  std::string topic = "(input:topic)";
  /**
   * Provides the possibility to define custom default values for the ports implemented by `class_name`. This will
   * override the "hard-coded" value and allows for configuring a behavior tree node without touching its source file.
   */
  std::map<std::string, std::string> port_defaults = {};
  /// Period [s] (measured from tree construction) after the server is considered unreachable.
  std::chrono::duration<double> wait_timeout = std::chrono::duration<double>(3);
  /// Period [s] (measured from sending a goal request) after the node aborts waiting for a server response.
  std::chrono::duration<double> request_timeout = std::chrono::duration<double>(2);
  /// Flag whether to tolerate if the action/service is unreachable when trying to create the client. If set to
  /// `true`, a warning is logged. Otherwise, an exception is raised.
  bool allow_unreachable = false;
  /// Minimum ROS 2 logging severity level for this particular node. Empty means to inherit the parent logging severity.
  std::string logger_level = "";
  /// Flexible YAML node which allows providing additional and customized registration options to the behavior tree node
  /// implementation.
  YAML::Node extra;

  /**
   * @brief Verify that the options are valid (e.g. all required values are set).
   * @return `true` if valid, `false` otherwise.
   */
  bool valid() const;
};

}  // namespace auto_apms_behavior_tree::core

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond INTERNAL
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::core::NodeRegistrationOptions>
{
  using Options = auto_apms_behavior_tree::core::NodeRegistrationOptions;
  inline static Node encode(const Options & rhs)
  {
    Node node(NodeType::Map);
    node[Options::PARAM_NAME_CLASS] = rhs.class_name;
    node[Options::PARAM_NAME_DESCRIPTION] = rhs.description;
    node[Options::PARAM_NAME_ROS2TOPIC] = rhs.topic;
    node[Options::PARAM_NAME_DEFAULTS] = rhs.port_defaults;
    node[Options::PARAM_NAME_WAIT_TIMEOUT] = rhs.wait_timeout.count();
    node[Options::PARAM_NAME_REQUEST_TIMEOUT] = rhs.request_timeout.count();
    node[Options::PARAM_NAME_ALLOW_UNREACHABLE] = rhs.allow_unreachable;
    node[Options::PARAM_NAME_LOGGER_LEVEL] = rhs.logger_level;
    node[Options::PARAM_NAME_EXTRA] = rhs.extra;
    return node;
  }
  inline static bool decode(const Node & node, Options & rhs)
  {
    if (!node.IsMap())
      throw auto_apms_util::exceptions::YAMLFormatError(
        "YAML::Node for auto_apms_behavior_tree::core::NodeRegistrationOptions must be map but is type " +
        std::to_string(node.Type()) + " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      const Node val = it->second;

      if (key == Options::PARAM_NAME_EXTRA) {
        // Any valid yaml is allowed as extra options
        rhs.extra = val;
        continue;
      }

      if (key == Options::PARAM_NAME_DEFAULTS) {
        if (!val.IsMap()) {
          throw auto_apms_util::exceptions::YAMLFormatError(
            "Value for key '" + key + "' must be a map but is type " + std::to_string(val.Type()) +
            " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
        }
        rhs.port_defaults = val.as<std::map<std::string, std::string>>();
        continue;
      }

      // The following options may only be scalar
      if (!val.IsScalar()) {
        throw auto_apms_util::exceptions::YAMLFormatError(
          "Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
          " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");
      }
      if (key == Options::PARAM_NAME_CLASS) {
        rhs.class_name = val.as<std::string>();
        continue;
      }
      if (key == Options::PARAM_NAME_DESCRIPTION) {
        rhs.description = val.as<std::string>();
        continue;
      }
      if (key == Options::PARAM_NAME_ROS2TOPIC) {
        rhs.topic = val.as<std::string>();
        continue;
      }
      if (key == Options::PARAM_NAME_WAIT_TIMEOUT) {
        rhs.wait_timeout = std::chrono::duration<double>(val.as<double>());
        continue;
      }
      if (key == Options::PARAM_NAME_REQUEST_TIMEOUT) {
        rhs.request_timeout = std::chrono::duration<double>(val.as<double>());
        continue;
      }
      if (key == Options::PARAM_NAME_ALLOW_UNREACHABLE) {
        rhs.allow_unreachable = val.as<bool>();
        continue;
      }
      if (key == Options::PARAM_NAME_LOGGER_LEVEL) {
        rhs.logger_level = val.as<std::string>();
        continue;
      }

      // Unkown parameter
      throw auto_apms_util::exceptions::YAMLFormatError("Unkown parameter name '" + key + "'.");
    }
    return true;
  }
};

}  // namespace YAML
/// @endcond
