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

#include "auto_apms_behavior_tree/builder/tree_builder_factory_template.hpp"
#include "pluginlib/class_list_macros.hpp"

/**
 * @ingroup auto_apms_behavior_tree
 * @brief Macro for registering a behavior tree builder plugin which may be loaded at runtime to create a
 * behavior tree according to the implementation.
 *
 * Builders are created by deriving from auto_apms_behavior_tree::TreeBuilderBase and implementing the
 * virtual methods as desired.
 *
 * @param type Fully qualified name of the class.
 */
#define AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILDER(type)                                                                 \
  PLUGINLIB_EXPORT_CLASS(auto_apms_behavior_tree::TreeBuilderFactoryTemplate<type>,                                    \
                         auto_apms_behavior_tree::TreeBuilderFactoryInterface)
