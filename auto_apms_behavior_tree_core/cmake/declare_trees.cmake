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

#
# Declare behavior trees to the package's resources.
#
# This macro makes behavior trees available to the ROS 2 workspace by
# exposing the corresponding XML and node manifest files as ament_index
# resources.
#
# :param ARGN: Behavior tree XML files to be added to this package's resources.
# :type ARGN: list of files
# :param NODE_MANIFEST: One or more relative paths or existing resource identities of node manifests.
#   If specified, behavior tree nodes associated with this manifest can be
#   loaded automatically and are available for every tree under ARGN.
# :type NODE_MANIFEST: list of strings
#
# @public
#
macro(auto_apms_behavior_tree_declare_trees)

    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs NODE_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    foreach(_path_argument ${ARGS_UNPARSED_ARGUMENTS})
        # Check if behavior tree file exists
        get_filename_component(_tree_abs_path__source "${_path_argument}" REALPATH)
        if(NOT EXISTS "${_tree_abs_path__source}")
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_declare_trees(): Behavior tree file ${_tree_abs_path__source} does not exist."
            )
        endif()

        # Verify that the file hasn't been registered.
        if("${_path_argument}" IN_LIST _package_tree_file_abs_paths__source)
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_declare_trees(): Behavior tree file ${_path_argument} has already been registered."
            )
        endif()
        list(APPEND _package_tree_file_abs_paths__source "${_path_argument}")

        get_filename_component(_tree_file_name "${_tree_abs_path__source}" NAME)
        get_filename_component(_tree_file_stem "${_tree_abs_path__source}" NAME_WE)

        # Collect all available behavior tree IDs
        file(READ "${_tree_abs_path__source}" _tree_file_content)
        string(REGEX MATCHALL "<BehaviorTree ID=\"[A-Za-z0-9_]+\">" _matches "${_tree_file_content}")
        if("${_matches}" STREQUAL "")
            message(
                FATAL_ERROR
                "auto_apms_behavior_tree_declare_trees(): Behavior tree file ${_tree_abs_path__source} doesn't specify any valid behavior trees."
            )
        endif()
        set(_tree_file_tree_names "")
        foreach(_match ${_matches})
            string(REGEX MATCH "<BehaviorTree ID=\"([A-Za-z0-9_]+)\">" _ "${_match}")
            set(_tree_name ${CMAKE_MATCH_1})
            # Verify no duplicate tree IDs inside file
            if("${_tree_name}" IN_LIST _tree_file_tree_names)
                message(
                    FATAL_ERROR
                    "auto_apms_behavior_tree_declare_trees(): Behavior tree with name '${_tree_name}' exists multiple times in file ${_path_argument}."
                )
            endif()
            list(APPEND _tree_file_tree_names "${_tree_name}")
        endforeach()

        # Prevent ambiguous tree identities
        set(_index 0)
        foreach(_stem ${_package_tree_file_stems})
            # Check if a tree with the same stem has been registered before.
            if("${_stem}" STREQUAL "${_tree_file_stem}")
                # Check if this tree registers trees with the same name
                list(GET _package_tree_names "${_index}" _names_associated_with_stem)
                string(REPLACE "|" ";" _names_associated_with_stem "${_names_associated_with_stem}")
                foreach(_name ${_names_associated_with_stem})
                    list(FIND _tree_file_tree_names "${_name}" _index2)
                    if(_index2 GREATER -1)
                        message(
                            FATAL_ERROR
                            "auto_apms_behavior_tree_declare_trees(): Found tree name '${_name}' in behavior tree file ${_path_argument} which has been registered by a file with the same name before. This would create an ambigious resource identity (${PROJECT_NAME}::${_stem}::${_name})."
                        )
                    endif()
                endforeach()
            endif()
            math(EXPR _index "${_index} + 1")
        endforeach()

        # Store file stem and associated tree names
        set(_temp_names "${_tree_file_tree_names}")
        string(REPLACE ";" "|" _temp_names "${_temp_names}")
        list(APPEND _package_tree_names "${_temp_names}")
        list(APPEND _package_tree_file_stems "${_tree_file_stem}")

        set(_tree_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__TREE}")
        set(_node_manifest_rel_paths__install "") # Empty string if no manifest is given

        if(NOT "${ARGS_NODE_MANIFEST}" STREQUAL "")

            set(_existing_metadata_ids__build "")
            set(_existing_node_manifest_abs_paths__build "")
            set(_existing_metadata_ids__resources "")
            set(_existing_node_manifest_abs_paths__resources "")

            # Collect metadata generated by this package
            foreach(_tuple ${_AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_BUILD_INFO})
                string(REPLACE "@" ";" _tuple "${_tuple}")
                list(GET _tuple 0 _id)
                list(GET _tuple 1 _path)
                list(APPEND _existing_metadata_ids__build "${PROJECT_NAME}::${_id}")
                list(APPEND _existing_node_manifest_abs_paths__build "${_path}")
            endforeach()

            # Collect metadata ids registered by other packages
            ament_index_get_resources(_packages_with_node_manifest_resource "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST}")
            foreach(_package ${_packages_with_node_manifest_resource})
                # Skip resources installed by this package since we use the build info variable instead
                if("${_package}" STREQUAL "${PROJECT_NAME}")
                    continue()
                endif()
                ament_index_get_resource(_content "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST}" "${_package}")
                string(REPLACE "\n" ";" _content "${_content}")
                foreach(_tuple ${_content})
                    string(REPLACE "|" ";" _tuple "${_tuple}")
                    list(GET _tuple 0 _id)
                    list(GET _tuple 1 _rel_path)
                    ament_index_has_resource(_prefix "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_TYPE_NAME__NODE_MANIFEST}" "${_package}")
                    list(APPEND _existing_metadata_ids__resources "${_package}::${_id}")
                    list(APPEND _existing_node_manifest_abs_paths__resources "${_prefix}/${_rel_path}")
                endforeach()
            endforeach()

            # Change the input to the auto_apms_behavior_tree_generate_node_metadata macro call so that we use all existing metadata
            set(_generate_metadata_inputs ${ARGS_NODE_MANIFEST})
            set(_matching_existing_metadata_count 0)
            foreach(_var ${ARGS_NODE_MANIFEST})
                # Check wether the user provided a metadata id referring to this package or another (Using the '::' delimiter)
                string(FIND "${_var}" "::" _index)
                if("${_index}" GREATER -1)
                    # Check if metadata for _var has already been generated by this package
                    list(FIND _existing_metadata_ids__build "${_var}" _index)
                    if("${_index}" GREATER -1)
                        list(GET _existing_node_manifest_abs_paths__build "${_index}" _path)
                    else()
                        # Check if metadata for _var has been generated by other packages
                        list(FIND _existing_metadata_ids__resources "${_var}" _index)
                        if("${_index}" GREATER -1)
                            list(GET _existing_node_manifest_abs_paths__resources "${_index}" _path)
                        else()
                            message(
                                FATAL_ERROR
                                "auto_apms_behavior_tree_declare_trees(): Metadata ID '${_var}' was provided under NODE_MANIFEST, but no such resource can be found."
                            )
                        endif()
                    endif()


                    # If matching metadata is generated by this package, replace the corresponding item in the inputs
                    # with the existing node manifest build path
                    list(FIND _generate_metadata_inputs "${_var}" _index)
                    list(REMOVE_AT _generate_metadata_inputs "${_index}")
                    list(INSERT _generate_metadata_inputs "${_index}" "${_path}")
                    math(EXPR _matching_existing_metadata_count "${_matching_existing_metadata_count} + 1")
                else()
                    # If the '::' delimiter wasn't used, the user must provide a relative path to an existing node manifest file
                    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_var}")
                        message(
                            FATAL_ERROR
                            "auto_apms_behavior_tree_declare_trees(): '${_var}' (provided under NODE_MANIFEST) is interpreted as a relative path to a node manfiest, but no such file can be found relative to the current source directory ${CMAKE_CURRENT_SOURCE_DIR}."
                        )
                    endif()
                endif()
            endforeach()

            # Is there any existing metadata registered under the given metadata ID(s)?
            if("${_matching_existing_metadata_count}" EQUAL 0)
                # We cannot use any existing node manifests and must generate everything
                auto_apms_behavior_tree_generate_node_metadata("${_tree_file_stem}" ${ARGS_NODE_MANIFEST})

                # Sticking to the default manifest install file path for the resource info
                set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_tree_file_stem}.yaml")
            else()
                list(LENGTH _generate_metadata_inputs _manifest_args_length)
                if("${_manifest_args_length}" EQUAL "${_matching_existing_metadata_count}")
                    # We don't have to generate anything because all required node manifests have already been generated under the provided ID.
                    foreach(_path ${_generate_metadata_inputs})
                        if("${_path}" IN_LIST _existing_node_manifest_abs_paths__build)
                            # Regarding the manifests generated by this package, there will be corresponding install paths and we just have to forward those.
                            get_filename_component(_file_name "${_path}" NAME)
                            list(APPEND _node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/${_file_name}")
                        else()
                            # Regarding the manifests generated by other package, we use the existing absolute manifest install paths.
                            list(APPEND _node_manifest_rel_paths__install "${_path}")
                        endif()
                    endforeach()
                else()
                    # There are some already registered node manifests (looked up using the provided metadata ID(s)) but we also have to generate additional metadata. So we look up the existing node manifest files and use them to generate the metadata under a new ID.
                    auto_apms_behavior_tree_generate_node_metadata("${_tree_file_stem}" ${_generate_metadata_inputs})

                    # Sticking to the default manifest install file path for the resource info
                    set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_tree_file_stem}.yaml")
                endif()
            endif()
        endif()

        # Install behavior tree
        install(
            FILES "${_tree_abs_path__source}"
            DESTINATION "${_tree_rel_dir__install}")

        # Fill resource info
        set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}${_tree_file_stem}|${_tree_file_tree_names}|${_tree_rel_dir__install}/${_tree_file_name}|${_node_manifest_rel_paths__install}\n")
    endforeach()

endmacro()
