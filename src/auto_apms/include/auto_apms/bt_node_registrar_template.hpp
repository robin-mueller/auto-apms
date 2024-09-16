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

#include <boost/core/demangle.hpp>

#include "auto_apms/bt_node_registrar.hpp"

namespace auto_apms {

template <
    typename BTNodeType,
    bool requires_ros_node_params = std::
        is_constructible<BTNodeType, const std::string &, const BT::NodeConfig &, const BT::RosNodeParams &>::value>
class BTNodeRegistrarTemplate : public BTNodeRegistrar
{
   public:
    BTNodeRegistrarTemplate() = default;
    virtual ~BTNodeRegistrarTemplate() = default;

    bool RequiresROSNodeParams() override { return requires_ros_node_params; }

    void RegisterForBehaviorTreeFactory(BT::BehaviorTreeFactory &factory,
                                        const std::string &registration_name,
                                        const BT::RosNodeParams *const params_ptr = nullptr) override
    {
        if constexpr (requires_ros_node_params) {
            if (!params_ptr) {
                throw std::runtime_error(
                    boost::core::demangle(typeid(BTNodeType).name()) +
                    " requires to pass a valid BT::RosNodeParams object via argument 'params_ptr'");
            }
            factory.registerNodeType<BTNodeType>(registration_name, *params_ptr);
        }
        else {
            factory.registerNodeType<BTNodeType>(registration_name);
        }
    }
};
}  // namespace auto_apms
