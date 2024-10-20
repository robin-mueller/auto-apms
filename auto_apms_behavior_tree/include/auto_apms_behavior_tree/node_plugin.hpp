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

#include "auto_apms_behavior_tree/node_plugin_base.hpp"

// Include base classes for inheritance in downstream source files
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

namespace auto_apms_behavior_tree {

template <
    typename BTNodeType,
    bool requires_ros_node_params = std::
        is_constructible<BTNodeType, const std::string &, const BT::NodeConfig &, const BT::RosNodeParams &>::value>
class BTNodePlugin : public BTNodePluginBase
{
   public:
    BTNodePlugin() = default;
    virtual ~BTNodePlugin() = default;

    bool RequiresROSNodeParams() const override { return requires_ros_node_params; }

    void RegisterWithBehaviorTreeFactory(BT::BehaviorTreeFactory &factory,
                                         const std::string &registration_name,
                                         const BT::RosNodeParams *const params_ptr = nullptr) const override
    {
        if constexpr (requires_ros_node_params) {
            if (!params_ptr) {
                throw std::runtime_error(
                    boost::core::demangle(typeid(BTNodeType).name()) +
                    " requires a valid BT::RosNodeParams object to be passed via argument 'params_ptr'.");
            }
            factory.registerNodeType<BTNodeType>(registration_name, *params_ptr);
        }
        else {
            factory.registerNodeType<BTNodeType>(registration_name);
        }
    }
};

}  // namespace auto_apms_behavior_tree

#include "pluginlib/class_list_macros.hpp"

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Macro for registering a behavior tree node plugin.
 * @param class_type The fully qualified class name.
 */
#define AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(class_type) \
    PLUGINLIB_EXPORT_CLASS(auto_apms_behavior_tree::BTNodePlugin<class_type>, auto_apms_behavior_tree::BTNodePluginBase)
