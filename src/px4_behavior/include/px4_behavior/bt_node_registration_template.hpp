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

#include "px4_behavior/bt_node_registration.hpp"

namespace px4_behavior {

template <typename BTNodeType>
class BTNodeRegistrationTemplate : public BTNodeRegistration
{
   public:
    BTNodeRegistrationTemplate() = default;
    virtual ~BTNodeRegistrationTemplate() = default;

    void RegisterForBehaviorTreeFactory(BT::BehaviorTreeFactory &factory,
                                        const std::string &type_name,
                                        const BT::RosNodeParams &params) override
    {
        constexpr bool needs_ros_params = std::
            is_constructible<BTNodeType, const std::string &, const BT::NodeConfig &, const BT::RosNodeParams &>::value;

        if constexpr (needs_ros_params) { factory.registerNodeType<BTNodeType>(type_name, params); }
        else {
            factory.registerNodeType<BTNodeType>(type_name);
        }
    }
};
}  // namespace px4_behavior
