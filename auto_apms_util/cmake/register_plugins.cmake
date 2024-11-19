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

# Macro that registers pluginlib plugins from a specific target.
# If FACTORY_TEMPLATE_CLASS is specified, the plugin class must be designed to be created using a factory template helper class,
# since the plugin's type will be registered as FactoryTemplateHelper<MyClass>.
macro(auto_apms_util_register_plugins target base_class)

    if(NOT TARGET ${target})
        message(
        FATAL_ERROR
        "auto_apms_util_register_plugins(): '${target}' is not a target.")
    endif()

    # Check target type
    get_target_property(_target_type ${target} TYPE)
    if(NOT _target_type STREQUAL "SHARED_LIBRARY")
        message(
        FATAL_ERROR
        "auto_apms_util_register_plugins(): '${target}' is not a shared library target.")
    endif()

    # Parse arguments
    set(options "")
    set(oneValueArgs FACTORY_TEMPLATE_CLASS)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    foreach(_class_name ${ARGS_UNPARSED_ARGUMENTS})
        if("${_class_name}" IN_LIST _AUTO_APMS_UTIL__PLUGIN_CLASS_NAMES)
            message(
            FATAL_ERROR
            "auto_apms_util_register_plugins(): Class name '${_class_name}' has already been registered before.")
        endif()

        # Append all class names to a list to keep track of all registered classes
        list(APPEND _AUTO_APMS_UTIL__PLUGIN_CLASS_NAMES "${_class_name}")

        # Append to the variable that holds the content of the pluginlib plugins.xml file
        if("${ARGS_FACTORY_TEMPLATE_CLASS}" STREQUAL "")
            set(_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT "${_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT}<library path=\"${target}\"><class name=\"${_class_name}\" type=\"${_class_name}\" base_class_type=\"${base_class}\" /></library>\n")
        else()
            set(_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT "${_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT}<library path=\"${target}\"><class name=\"${_class_name}\" type=\"${ARGS_FACTORY_TEMPLATE_CLASS}<${_class_name}>\" base_class_type=\"${base_class}\" /></library>\n")
        endif()
    endforeach()

endmacro()