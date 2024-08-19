#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_ros2/ros_node_params.hpp>

namespace px4_behavior {
class BTNodeFactory
{
   public:
    BTNodeFactory() = default;
    virtual ~BTNodeFactory() = default;

    virtual void RegisterForBehaviorTreeFactory(BT::BehaviorTreeFactory& factory,
                                                   const std::string& type_name,
                                                   const BT::RosNodeParams& params) = 0;
};
}  // namespace px4_behavior