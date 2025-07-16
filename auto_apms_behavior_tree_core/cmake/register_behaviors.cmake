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
# (behavior build requests) at runtime. These definitions are a central concept in AutoAPMS. Essentially, they act as
# blueprints for building and initializing behaviors in ROS 2.
#
# ARGN can contain relative or absolute file paths. The file contents act as instructions for building a behavior given
# to the build handler specified under BUILD_HANDLER.
#
# ARGN can also contain simple strings. Such arguments are directly forwarded to the behavior build handler specified
# under BUILD_HANDLER. This only makes sense if the build handler is able to interpret the string correctly.
#
# :param build_request: Relative path to a file containing a behavior definition or a simple string. This argument
#    determines the build request given to the behavior build handler provided with BUILD_HANDLER. The user must
#    make sure that the given build handler is able to interpret the given request.
# :type build_request: string
# :param BUILD_HANDLER: Fully qualified class name of the behavior tree build handler that should be
#    used by default to interpret the given behaviors.
# :type BUILD_HANDLER: string
# :param CATEGORY: Optional category name to which the behaviors belong.
#    If omitted, the default category is used.
# :type CATEGORY: string
# :param ALIAS: Optional name for the behavior resource(s). If ARGN contains multiple arguments, a number is
#    appended to the alias. However, it is best practice to only use this keyword in combination with a single behavior.
#    If omitted, the file stem respectively the simple string is used as a behavior's alias.
# :type ALIAS: string
# :param ENTRYPOINT: Single point of entry for behavior execution. For behavior trees, this is the name of the root tree,
#    but for other categories, this may interpreted differently.
# :type ENTRYPOINT: string
# :param NODE_MANIFEST: One or more relative paths or resource identities of existing node manifests.
#   If specified, behavior tree nodes associated with this manifest can be
#   loaded automatically and are available for every behavior registered with this macro call.
# :type NODE_MANIFEST: list of strings
#
# @public
#
macro(auto_apms_behavior_tree_register_behavior build_request)

    # Parse arguments
    set(options "")
    set(oneValueArgs BUILD_HANDLER CATEGORY ALIAS ENTRYPOINT)
    set(multiValueArgs NODE_MANIFEST)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT DEFINED ARGS_BUILD_HANDLER)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behavior(): The BUILD_HANDLER keyword is required. You must specify the fully qualified class name of the default behavior tree build handler used to create the behavior from the given definitions"
        )
    endif()
    if(NOT DEFINED ARGS_CATEGORY)
        # Default
        set(ARGS_CATEGORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__DEFAULT_BEHAVIOR_CATEGORY}")
    endif()
    if(NOT DEFINED ARGS_ENTRYPOINT)
        # Default
        set(ARGS_ENTRYPOINT "")
    endif()

    # Check if category is valid
    string(REGEX MATCH "[^A-Za-z0-9_-]" _has_illegal "${ARGS_CATEGORY}")
    if(_has_illegal)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behavior(): Category '${ARGS_CATEGORY}' contains illegal characters. Only alphanumeric, '_' and '-' are allowed."
        )
    endif()

    # Check if entrypoint is valid
    string(REGEX MATCH "[^A-Za-z0-9_-]" _has_illegal "${ARGS_ENTRYPOINT}")
    if(_has_illegal)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behavior(): Entrypoint '${ARGS_ENTRYPOINT}' contains illegal characters. Only alphanumeric, '_' and '-' are allowed."
        )
    endif()

    set(_behavior_file_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__BEHAVIOR}")

    get_filename_component(_behavior_file_abs_path__source "${build_request}" REALPATH)
    get_filename_component(_behavior_alias "${build_request}" NAME_WE) # Simply returns the string itself if it is not a file path

    # If provided, overwrite the alias for the behavior
    if(DEFINED ARGS_ALIAS)
        set(_behavior_alias "${ARGS_ALIAS}")
    endif()

    # Check if alias is valid
    string(REGEX MATCH "[^A-Za-z0-9_:-]" _has_illegal "${_behavior_alias}")
    if(_has_illegal)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behavior(): Behavior alias ${_behavior_alias} for '${build_request}' contains illegal characters. Only alphanumeric, '_', '-', and ':' are allowed. Make sure to change the macro's arguments or specify a valid alias."
        )
    endif()

    # Verify no duplicate behaviors
    if("${ARGS_CATEGORY}${_behavior_alias}" IN_LIST _all_behaviors)
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_register_behavior(): '${build_request}' is aliased with '${_behavior_alias}', but this alias was already used to register a behavior in category '${ARGS_CATEGORY}'. An alias must be unique per category and package. Use the ALIAS keyword to specify a unique alias for behaviors."
        )
    endif()
    list(APPEND _all_behaviors "${ARGS_CATEGORY}${_behavior_alias}")
    list(LENGTH _all_behaviors _behavior_num)

    set(_metadata_id "behavior${_behavior_num}")

    # Determine the build request
    if(EXISTS "${_behavior_file_abs_path__source}")
        get_filename_component(_file_name "${_behavior_file_abs_path__source}" NAME)

        # Track the behavior file so CMake knows it's an input dependency
        # Make sure to give the file a unique name
        set(_file_name__unique "${_metadata_id}_${_file_name}")
        configure_file(
            "${_behavior_file_abs_path__source}"
            "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_file_name__unique}"
            COPYONLY
        )

        # Install the file (must be the source file for symlinks to work correctly)
        install(
            FILES "${_behavior_file_abs_path__source}"
            DESTINATION "${_behavior_file_rel_dir__install}"
            RENAME "${_file_name__unique}"
        )

        # Fill build request with install path
        set(_build_request_field "${_behavior_file_rel_dir__install}/${_file_name__unique}")
    else()
        # Fill build request with raw string
        set(_build_request_field "${build_request}")
    endif()

    set(_node_manifest_rel_paths__install "") # Empty string if no manifest is given
    if(NOT "${ARGS_NODE_MANIFEST}" STREQUAL "")

        set(_existing_metadata_ids__build "")
        set(_existing_node_manifest_abs_paths__build "")
        set(_existing_metadata_ids__resources "")
        set(_existing_node_manifest_abs_paths__resources "")

        # Collect metadata generated by this package
        foreach(_tuple ${_AUTO_APMS_BEHAVIOR_TREE_CORE__NODE_MANIFEST_BUILD_INFO})
            string(REPLACE "@" ";" _tuple "${_tuple}")
            list(GET _tuple 0 _id)
            list(GET _tuple 1 _behavior_file_abs_path__source)
            list(APPEND _existing_metadata_ids__build "${PROJECT_NAME}::${_id}")
            list(APPEND _existing_node_manifest_abs_paths__build "${_behavior_file_abs_path__source}")
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

        # # # # # # # # #
        #
        # Change the input variables for the auto_apms_behavior_tree_generate_node_metadata macro called later
        # so that we use all existing metadata
        #
        set(_generate_metadata_inputs ${ARGS_NODE_MANIFEST})
        set(_matching_existing_metadata_count 0)
        foreach(_var ${ARGS_NODE_MANIFEST})
            # Check wether the user provided a metadata id referring to this package or another (Using the '::' delimiter)
            string(FIND "${_var}" "::" _index)
            if("${_index}" GREATER -1)
                # Check if metadata for _var has already been generated by this package
                list(FIND _existing_metadata_ids__build "${_var}" _index)
                if("${_index}" GREATER -1)
                    list(GET _existing_node_manifest_abs_paths__build "${_index}" _behavior_file_abs_path__source)
                else()
                    # Check if metadata for _var has been generated by other packages
                    list(FIND _existing_metadata_ids__resources "${_var}" _index)
                    if("${_index}" GREATER -1)
                        list(GET _existing_node_manifest_abs_paths__resources "${_index}" _behavior_file_abs_path__source)
                    else()
                        message(
                            FATAL_ERROR
                            "auto_apms_behavior_tree_register_behavior(): Metadata ID '${_var}' was provided under NODE_MANIFEST, but no such resource can be found."
                        )
                    endif()
                endif()


                # If matching metadata is generated by this package, replace the corresponding item in the inputs
                # with the existing node manifest build path
                list(FIND _generate_metadata_inputs "${_var}" _index)
                list(REMOVE_AT _generate_metadata_inputs "${_index}")
                list(INSERT _generate_metadata_inputs "${_index}" "${_behavior_file_abs_path__source}")
                math(EXPR _matching_existing_metadata_count "${_matching_existing_metadata_count} + 1")
            else()
                # If the '::' delimiter wasn't used, the user must provide a relative path to an existing node manifest file
                if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_var}")
                    message(
                        FATAL_ERROR
                        "auto_apms_behavior_tree_register_behavior(): '${_var}' (provided under NODE_MANIFEST) is interpreted as a relative path to a node manfiest, but no such file can be found relative to the current source directory ${CMAKE_CURRENT_SOURCE_DIR}."
                    )
                endif()
            endif()
        endforeach()
        #
        # # # # # # # # #

        # Call auto_apms_behavior_tree_generate_node_metadata reusing as much of the existing metadata as possible
        if("${_matching_existing_metadata_count}" EQUAL 0)
            # We cannot use any existing node manifests and must generate everything
            auto_apms_behavior_tree_generate_node_metadata("${_metadata_id}" ${ARGS_NODE_MANIFEST})

            # Sticking to the default manifest install file path for the resource info
            set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_metadata_id}.yaml")
        else()
            list(LENGTH _generate_metadata_inputs _manifest_args_length)
            if("${_manifest_args_length}" EQUAL "${_matching_existing_metadata_count}")
                # We don't have to generate anything because all required node manifests have already been generated under the provided ID.
                foreach(_behavior_file_abs_path__source ${_generate_metadata_inputs})
                    if("${_behavior_file_abs_path__source}" IN_LIST _existing_node_manifest_abs_paths__build)
                        # Regarding the manifests generated by this package, there will be corresponding install paths and we just have to forward those.
                        get_filename_component(_file_name "${_behavior_file_abs_path__source}" NAME)
                        list(APPEND _node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/${_file_name}")
                    else()
                        # Regarding the manifests generated by other package, we use the existing absolute manifest install paths.
                        list(APPEND _node_manifest_rel_paths__install "${_behavior_file_abs_path__source}")
                    endif()
                endforeach()
            else()
                # There are some already registered node manifests (looked up using the provided metadata ID(s)) but we also have to generate additional metadata. So we look up the existing node manifest files and use them to generate the metadata under a new ID.
                auto_apms_behavior_tree_generate_node_metadata("${_metadata_id}" ${_generate_metadata_inputs})

                # Sticking to the default manifest install file path for the resource info
                set(_node_manifest_rel_paths__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}/node_manifest_${_metadata_id}.yaml")
            endif()
        endif()
    endif()

    # Populate resource file variable
    set(_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_FILE__BEHAVIOR}${ARGS_CATEGORY}|${_behavior_alias}|${ARGS_BUILD_HANDLER}|${_build_request_field}|${ARGS_ENTRYPOINT}|${_node_manifest_rel_paths__install}\n")

endmacro()
