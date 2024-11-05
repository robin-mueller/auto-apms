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

# Macro that registers behavior tree build directors from a specific target
macro(auto_apms_behavior_tree_register_builders target)

    if(NOT TARGET ${target})
        message(
        FATAL_ERROR
        "auto_apms_behavior_tree_register_builders(): '${target}' is not a target.")
    endif()

    # Check target type
    get_target_property(_target_type ${target} TYPE)
    if(NOT _target_type STREQUAL "SHARED_LIBRARY")
        message(
        FATAL_ERROR
        "auto_apms_behavior_tree_register_builders(): '${target}' is not a shared library target.")
    endif()

    cmake_parse_arguments(ARGS "" "" "" ${ARGN})

    # Parse behavior tree node class names
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
        if(${_class_name} IN_LIST _AUTO_APMS_BEHAVIOR_TREE__BUILDER_CLASS_NAMES)
            message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_builders(): Class name '${_class_name}' has already been registered before.")
        endif()

        # Append all class names to a list to keep track of all available behavior tree node classes within this package
        list(APPEND _AUTO_APMS_BEHAVIOR_TREE__BUILDER_CLASS_NAMES ${_class_name})

        # Append to the variable that holds the content of the pluginlib xml file
        set(_AUTO_APMS_BEHAVIOR_TREE__BUILDER_PLUGIN_XML_CONTENT "${_AUTO_APMS_BEHAVIOR_TREE__BUILDER_PLUGIN_XML_CONTENT}<library path=\"${target}\"><class name=\"${_class_name}\" type=\"auto_apms_behavior_tree::TreeBuilderFactoryTemplate<${_class_name}>\" base_class_type=\"auto_apms_behavior_tree::TreeBuilderFactoryInterface\" /></library>\n")
    endforeach()

endmacro()
