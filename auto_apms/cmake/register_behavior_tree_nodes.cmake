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

# Macro that registers a library target for a behavior tree node plugin
macro(auto_apms_register_behavior_tree_nodes plugin_lib_target)
    if(NOT TARGET ${plugin_lib_target})
        message(
        FATAL_ERROR
        "auto_apms_register_behavior_tree_nodes(): '${plugin_lib_target}' is not a target.")
    endif()

    # Check target type
    get_target_property(_target_type ${plugin_lib_target} TYPE)
    if(NOT _target_type STREQUAL "SHARED_LIBRARY")
        message(
        FATAL_ERROR
        "auto_apms_register_behavior_tree_nodes(): '${plugin_lib_target}' is not a shared library target.")
    endif()

    cmake_parse_arguments(ARGS "" "" "" ${ARGN})

    # Parse behavior tree node class names
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
        if(${_class_name} IN_LIST _AUTO_APMS_BEHAVIOR_TREE__NODE_CLASS_NAMES)
            message(
            FATAL_ERROR
            "auto_apms_register_behavior_tree_nodes(): Class name '${_class_name}' has already been registered before.")
        endif()

        # Append all class names to a list to keep track of all available behavior tree node classes within this package
        list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_CLASS_NAMES ${_class_name})

        # Append to the variable that holds the build information of the behavior tree node plugins (<class_name>@<library_path>)
        list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_BUILD_INFO "${_class_name}@$<TARGET_FILE:${plugin_lib_target}>")

        # Append to the variable that holds the content of the pluginlib xml file
        set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_XML_CONTENT "${_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_XML_CONTENT}<library path=\"${plugin_lib_target}\"><class name=\"${_class_name}\" type=\"auto_apms::detail::BTNodePlugin<${_class_name}>\" base_class_type=\"auto_apms::detail::BTNodePluginBase\" /></library>\n")
    endforeach()

endmacro()
