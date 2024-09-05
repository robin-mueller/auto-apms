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

#include "class_loader/class_loader.hpp"
#include "px4_behavior/bt_node_registration_template.hpp"

#define PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(BTNodeClass) \
    CLASS_LOADER_REGISTER_CLASS(px4_behavior::BTNodeRegistrationTemplate<BTNodeClass>, px4_behavior::BTNodeRegistration)
