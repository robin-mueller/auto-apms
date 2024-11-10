# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Macro that registers behavior tree node registration plugins from a specific target
macro(auto_apms_behavior_tree_register_nodes target)
    auto_apms_util_register_plugins(
        ${target}
        "auto_apms_behavior_tree::core::NodeRegistrationInterface"
        ${ARGN}
        FACTORY_TEMPLATE_CLASS "auto_apms_behavior_tree::core::NodeRegistrationTemplate"
    )

    # Append build information of the specified node plugins (<class_name>@<library_path>)
    foreach(_class_name ${ARGN})
        list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO "${_class_name}@$<TARGET_FILE:${target}>")
    endforeach()
endmacro()
