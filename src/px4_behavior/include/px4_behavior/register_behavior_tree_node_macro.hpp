#pragma once

#include <class_loader/class_loader.hpp>
#include <px4_behavior/bt_node_registration_template.hpp>

#define PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(BTNodeClass) \
    CLASS_LOADER_REGISTER_CLASS(px4_behavior::BTNodeRegistrationTemplate<BTNodeClass>, px4_behavior::BTNodeRegistration)
