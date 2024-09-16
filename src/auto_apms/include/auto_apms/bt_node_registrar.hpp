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

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace auto_apms {

class BTNodeRegistrar
{
   public:
    BTNodeRegistrar() = default;
    virtual ~BTNodeRegistrar() = default;

    virtual bool RequiresROSNodeParams() = 0;
    virtual void RegisterWithBehaviorTreeFactory(BT::BehaviorTreeFactory& factory,
                                                 const std::string& registration_name,
                                                 const BT::RosNodeParams* const params_ptr = nullptr) = 0;
};

}  // namespace auto_apms
