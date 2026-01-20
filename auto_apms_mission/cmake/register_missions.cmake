# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Register mission configuration file that will be discoverable using the identity <package_name>::<config_file_stem>
macro(auto_apms_mission_register_missions)

  # Parse arguments
  set(options "")
  set(oneValueArgs "")
  set(multiValueArgs "")
  cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  foreach(_rel_path ${ARGS_UNPARSED_ARGUMENTS})
    # Check mission config file exists
    get_filename_component(_path "${_rel_path}" REALPATH)
    if(NOT EXISTS "${_path}")
      message(
        FATAL_ERROR
        "auto_apms_mission_register_missions(): Mission config file ${_path} does not exist"
      )
    endif()
    get_filename_component(_name "${_path}" NAME)
    get_filename_component(_stem "${_path}" NAME_WE)

    # Verify no duplicates in file stems
    if("${_stem}" IN_LIST _all_mission_config_stems)
      message(
        FATAL_ERROR
        "auto_apms_mission_register_missions(): A mission config file with stem '${_stem}' was already registered"
      )
    endif()
    list(APPEND _all_mission_config_stems "${_stem}")

    auto_apms_behavior_tree_register_behavior(
      "${_path}"
      BUILD_HANDLER "auto_apms_mission::MissionFromStringBuildHandler"
      CATEGORY "${_AUTO_APMS_MISSION__DEFAULT_BEHAVIOR_CATEGORY__MISSION}"
    )
  endforeach()

endmacro()
