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

#include "auto_apms_behavior_tree/node/node_registration_interface.hpp"
#include <boost/core/demangle.hpp>

namespace auto_apms_behavior_tree
{

template <typename T,
          bool requires_ros_node_params =
              std::is_constructible<T, const std::string&, const BT::NodeConfig&, const RosNodeContext&>::value>
class NodeRegistrationTemplate : public NodeRegistrationInterface
{
public:
  NodeRegistrationTemplate() = default;
  virtual ~NodeRegistrationTemplate() = default;

  bool requiresROSNodeParams() const override
  {
    return requires_ros_node_params;
  }

  void registerWithBehaviorTreeFactory(BT::BehaviorTreeFactory& factory, const std::string& registration_name,
                                       const RosNodeContext* const params_ptr = nullptr) const override
  {
    if constexpr (requires_ros_node_params)
    {
      if (!params_ptr)
      {
        throw std::runtime_error(boost::core::demangle(typeid(T).name()) +
                                 " requires a valid RosNodeContext object to be passed via argument 'params_ptr'.");
      }
      factory.registerNodeType<T>(registration_name, *params_ptr);
    }
    else
    {
      factory.registerNodeType<T>(registration_name);
    }
  }
};

}  // namespace auto_apms_behavior_tree