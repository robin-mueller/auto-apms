#pragma once

#include "px4_behavior/bt_node_registration.hpp"

namespace px4_behavior {

template <typename BTNodeType>
class BTNodeRegistrationTemplate : public BTNodeRegistration
{
   public:
    BTNodeRegistrationTemplate() = default;
    virtual ~BTNodeRegistrationTemplate() = default;

    void RegisterForBehaviorTreeFactory(BT::BehaviorTreeFactory& factory,
                                        const std::string& type_name,
                                        const BT::RosNodeParams& params) override
    {
        constexpr bool needs_ros_params = std::
            is_constructible<BTNodeType, const std::string&, const BT::NodeConfig&, const BT::RosNodeParams&>::value;

        if constexpr (needs_ros_params)
            factory.registerNodeType<BTNodeType>(type_name, params);
        else
            factory.registerNodeType<BTNodeType>(type_name);
    }
};
}  // namespace px4_behavior
