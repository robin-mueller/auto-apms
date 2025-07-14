# Copyright 2025 Robin Müller
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

#
# Add AutoAPMS behaviors to the package's resources.
#
# This macro effectively registers files, adds them to the package's
# resources and provides a consistent way of grouping and discovering behavior definitions
# (behavior tree build requests) at runtime. They can later be used to instantiate behavior trees.
#
# If ARGN contains any strings that are not valid file paths, they are simply copied to the
# resource marker file. When used to instantiate behavior trees, the corresponding behavior tree
# build handler must be able to interpret the string correctly. This is typically used to refer to
# other resources by a specific identifier.
#
# :param ARGN: Relative paths to a file containing a behavior definition (must be interpretable by the
#    behavior tree build handler provided with BUILD_HANDLER) or a raw string (e.g. for referring to another resource).
# :type ARGN: list of strings
# :param BUILD_HANDLER: Fully qualified class name of the behavior tree build handler that should be
#    used by default to interpret the given behaviors.
# :type BUILD_HANDLER: string
# :param CATEGORY: Optional category name to which the behaviors belong.
#    If omitted, the default category is used.
# :type CATEGORY: string
#
# @public
#
macro(auto_apms_behavior_tree_register_behaviors)

    # Parse arguments
    set(options "")
    set(oneValueArgs BUILD_HANDLER CATEGORY)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT DEFINED ARGS_BUILD_HANDLER)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behaviors(): The BUILD_HANDLER keyword is required. You must specify the fully qualified class name of the default behavior tree build handler used to create the behavior from the given definitions"
        )
    endif()
    if(NOT DEFINED ARGS_BUILD_HANDLER)
        set(ARGS_CATEGORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY}")
    endif()

    set(_behavior_file_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__BEHAVIOR}")
    set(_behavior_file_abs_paths__source "")
    foreach(_arg ${ARGS_UNPARSED_ARGUMENTS})
        # Verify no duplicate behaviors
        if("${ARGS_CATEGORY}${_arg}" IN_LIST _all_args)
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_register_behaviors(): '${_arg}' was already used to register a behavior in category '${ARGS_CATEGORY}'. A behavior can only be registered once per category and package"
            )
        endif()
        list(APPEND _all_args "${ARGS_CATEGORY}${_arg}")

        # Check if arg is path or raw string
        get_filename_component(_path "${_arg}" REALPATH)
        if(EXISTS "${_path}")
            get_filename_component(_name "${_path}" NAME)

            # Append to the list of files to be installed
            list(APPEND _behavior_file_abs_paths__source "${_path}")

            # Fill resource info with install path
            set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR}${_behavior_file_rel_dir__install}/${_name}|${ARGS_BUILD_HANDLER}|${ARGS_CATEGORY}\n")
        else()
            string(REGEX MATCH "[^A-Za-z0-9_:-]" _has_illegal "${_arg}")
            if(_has_illegal)
                message(
                    FATAL_ERROR
                    "auto_apms_behavior_tree_register_behaviors(): '${_arg}' contains illegal characters. Only alphanumeric, '_', '-', and ':' are allowed"
                )
            endif()
            # Fill resource info with raw string
            set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR}${_arg}|${ARGS_BUILD_HANDLER}|${ARGS_CATEGORY}\n")
        endif()
    endforeach()

    # Install any provided files
    install(
        FILES ${_behavior_file_abs_paths__source}
        DESTINATION "${_behavior_file_rel_dir__install}"
    )

endmacro()
