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

#include <map>
#include <regex>
#include <stdexcept>
#include <type_traits>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_registration_interface.hpp"
#include "behaviortree_cpp/basic_types.h"

namespace auto_apms_behavior_tree::core
{

/// @cond INTERNAL

/**
 * @brief Wrapper that encapsulates a registration function for the behavior tree node class provided using template
 * argument @p T
 * @tparam T Behavior tree node class.
 */
template <
  class T, typename = std::enable_if_t<std::is_base_of_v<BT::TreeNode, T>>,
  bool requires_ros_node_params =
    std::is_constructible_v<T, const std::string &, const BT::NodeConfig &, RosNodeContext>>
class NodeRegistrationTemplate : public NodeRegistrationInterface
{
public:
  NodeRegistrationTemplate() = default;
  virtual ~NodeRegistrationTemplate() = default;

  bool requiresRosNodeContext() const override { return requires_ros_node_params; }

  void registerWithBehaviorTreeFactory(
    BT::BehaviorTreeFactory & factory, const std::string & registration_name,
    const RosNodeContext * const context_ptr = nullptr) const override
  {
    BT::PortsList ports_list = BT::getProvidedPorts<T>();
    if (context_ptr) {
      try {
        context_ptr->modifyProvidedPortsListForRegistration(ports_list);
      } catch (const std::exception & e) {
        throw exceptions::NodeRegistrationError(
          "[registerWithBehaviorTreeFactory] Error registering node '" + registration_name + "': " + e.what());
      }
    }

    if constexpr (requires_ros_node_params) {
      if (!context_ptr) {
        throw std::invalid_argument(
          "[registerWithBehaviorTreeFactory] Error registering node '" + registration_name +
          "': You must pass a valid RosNodeContext object to be passed via argument "
          "'context_ptr'.");
      }
      factory.registerNodeType<T>(registration_name, ports_list, *context_ptr);
    } else {
      if (context_ptr) {
        // Warn the user that some features of RosNodeContext are not supported
        if (context_ptr->registration_options_.port_alias.size() > 0) {
          throw exceptions::NodeRegistrationError(
            "[registerWithBehaviorTreeFactory] Error registering node '" + registration_name +
            "': Port aliasing is not supported by this node, but " + NodeRegistrationOptions::PARAM_NAME_PORT_ALIAS +
            " was provided in the registration options, please remove the field "
            "from the node manifest to avoid confusion.");
        }
      }
      factory.registerNodeType<T>(registration_name, ports_list);
    }
  }
};

/// @endcond

}  // namespace auto_apms_behavior_tree::core