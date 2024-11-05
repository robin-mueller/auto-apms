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

#include <string>
#include <chrono>
#include <map>

#include "auto_apms_util/yaml.hpp"

namespace auto_apms_behavior_tree
{
struct NodeRegistrationParams;
}

/// @cond
namespace YAML
{
template <>
struct convert<auto_apms_behavior_tree::NodeRegistrationParams>
{
  using Params = auto_apms_behavior_tree::NodeRegistrationParams;
  static Node encode(const Params& rhs);
  static bool decode(const Node& node, Params& lhs);
};
}  // namespace YAML
/// @endcond

namespace auto_apms_behavior_tree
{

/**
 * @brief Necessary parameters for loading and registering a behavior tree node class from a shared library using
 * e.g. NodeRegistrationClassLoader.
 */
struct NodeRegistrationParams
{
  static const std::string PARAM_NAME_CLASS;
  static const std::string PARAM_NAME_PORT;
  static const std::string PARAM_NAME_WAIT_TIMEOUT;
  static const std::string PARAM_NAME_REQUEST_TIMEOUT;

  AUTO_APMS_DEFINE_YAML_INTERPRETER_METHODS(NodeRegistrationParams)

  std::string class_name;
  std::string port;
  std::chrono::duration<double> wait_timeout = std::chrono::duration<double>(3);
  std::chrono::duration<double> request_timeout = std::chrono::duration<double>(1.5);
};

}  // namespace auto_apms_behavior_tree

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

/// @cond
namespace YAML
{
inline Node convert<auto_apms_behavior_tree::NodeRegistrationParams>::encode(const Params& rhs)
{
  Node node;
  node[Params::PARAM_NAME_CLASS] = rhs.class_name;
  node[Params::PARAM_NAME_PORT] = rhs.port;
  node[Params::PARAM_NAME_WAIT_TIMEOUT] = rhs.wait_timeout.count();
  node[Params::PARAM_NAME_REQUEST_TIMEOUT] = rhs.request_timeout.count();
  return node;
}
inline bool convert<auto_apms_behavior_tree::NodeRegistrationParams>::decode(const Node& node, Params& lhs)
{
  if (!node.IsMap())
    throw std::runtime_error("YAML::Node for auto_apms_behavior_tree::NodeRegistrationParams must be map but is type " +
                             std::to_string(node.Type()) +
                             " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

  for (auto it = node.begin(); it != node.end(); ++it)
  {
    const std::string key = it->first.as<std::string>();
    const Node& val = it->second;
    if (!val.IsScalar())
      throw std::runtime_error("Value for key '" + key + "' must be scalar but is type " + std::to_string(val.Type()) +
                               " (0: Undefined - 1: Null - 2: Scalar - 3: Sequence - 4: Map).");

    if (key == Params::PARAM_NAME_CLASS)
    {
      lhs.class_name = val.as<std::string>();
      continue;
    }
    if (key == Params::PARAM_NAME_PORT)
    {
      lhs.port = val.as<std::string>();
      continue;
    }
    if (key == Params::PARAM_NAME_WAIT_TIMEOUT)
    {
      lhs.wait_timeout = std::chrono::duration<double>(val.as<double>());
      continue;
    }
    if (key == Params::PARAM_NAME_REQUEST_TIMEOUT)
    {
      lhs.request_timeout = std::chrono::duration<double>(val.as<double>());
      continue;
    }
    // Unkown parameter
    throw std::runtime_error("Unkown parameter name '" + key + "'.");
  }
  return true;
}
}  // namespace YAML
/// @endcond
