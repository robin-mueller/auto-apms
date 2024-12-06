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

function(target_links_against target library_name result_var)
    # Get direct dependencies (LINK_LIBRARIES and INTERFACE_LINK_LIBRARIES)
    get_target_property(direct_libraries ${target} LINK_LIBRARIES)
    if(NOT direct_libraries)
        set(direct_libraries "")
    endif()

    get_target_property(interface_libraries ${target} INTERFACE_LINK_LIBRARIES)
    if(NOT interface_libraries)
        set(interface_libraries "")
    endif()

    list(APPEND all_libraries ${direct_libraries} ${interface_libraries})

    set(found FALSE)

    # Check each direct dependency
    foreach(lib ${all_libraries})
        if(lib STREQUAL ${library_name})
            set(found TRUE)
            break()
        endif()
    endforeach()

    set(${result_var} ${found} PARENT_SCOPE)
endfunction()

macro(auto_apms_behavior_tree_generate_node_metadata metadata_id)

    # Parse arguments
    set(options "")
    set(oneValueArgs MODEL_HEADER_TARGET GENERATED_MANIFEST_FILE_NAME GENERATED_MODEL_FILE_NAME)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if("${ARGS_UNPARSED_ARGUMENTS}" STREQUAL "")
        message(
            FATAL_ERROR
            "auto_apms_behavior_tree_generate_node_metadata(): You didn't provide any node manifest yaml files."
        )
    endif()

    if(metadata_id IN_LIST _AUTO_APMS_BEHAVIOR_TREE_CORE__METADATA_IDS)
        message(
        FATAL_ERROR
        "auto_apms_behavior_tree_generate_node_metadata(): Metadata with ID '${metadata_id}' has already been generated before.")
    endif()

    # Append metadata id to a list to keep track of all registered ids
    list(APPEND _AUTO_APMS_BEHAVIOR_TREE_CORE__METADATA_IDS "${metadata_id}")

    # Default names
    set(_generated_node_manifest_file_name "node_manifest_${metadata_id}.yaml")
    set(_generated_node_model_file_name "node_model_${metadata_id}.xml")

    if(NOT "${ARGS_GENERATED_MANIFEST_FILE_NAME}" STREQUAL "")
        get_filename_component(_generated_node_manifest_file_stem "${ARGS_GENERATED_MANIFEST_FILE_NAME}" NAME_WE)
        set(_generated_node_manifest_file_name "${_generated_node_manifest_file_stem}.yaml")
    endif()

    if(NOT "${ARGS_GENERATED_MODEL_FILE_NAME}" STREQUAL "")
        get_filename_component(_generated_node_model_file_stem "${ARGS_GENERATED_MODEL_FILE_NAME}" NAME_WE)
        set(_generated_node_model_file_name "${_generated_node_model_file_stem}.xml")
    endif()

    list(REMOVE_DUPLICATES ARGS_UNPARSED_ARGUMENTS) # Disregard duplicate manifest files
    file(MAKE_DIRECTORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}") # Create build directory if not existing

    # Replace potential CMake variable names with their values inside the manifest yaml file
    set(_create_node_manifest_input_file_paths__absolute "")
    foreach(_input_manifest_file_path ${ARGS_UNPARSED_ARGUMENTS})
        get_filename_component(_input_manifest_file_stem "${_input_manifest_file_path}" NAME_WE)
        set(_path "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/create_node_manifest_inputs__${metadata_id}/${_input_manifest_file_stem}.yaml")
        configure_file("${_input_manifest_file_path}" "${_path}")
        list(APPEND _create_node_manifest_input_file_paths__absolute "${_path}")
    endforeach()

    set(_generated_node_manifest_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}")
    set(_generated_node_model_rel_dir__install "${_AUTO_APMS_BEHAVIOR_TREE_CORE__RESOURCE_DIR_RELATIVE__METADATA}")

    # Create a valid metadata_id for the custom target
    string(REPLACE " " "_" _custom_target_suffix "${metadata_id}")
    string(TOLOWER "${_custom_target_suffix}" _custom_target_suffix)

    # Create the complete manifest for model generation.
    # Simultaneously, define a variable containing the library paths and generator expressions for node plugin dependencies.
    set(_generated_node_manifest_abs_path__build "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_generated_node_manifest_file_name}")
    execute_process(
        COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MANIFEST_CMD}"
            "${_create_node_manifest_input_file_paths__absolute}" # Paths of the manifest source files

            # General build information for node plugins compiled by this package.
            # Generator expressions cannot be evaluated yet since execute_process is handled at configuration time.
            # However, we don't require the correct library paths yet.
            "${_AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO}"

            "${PROJECT_NAME}"  # Name of the package that builds the behavior tree model
            "${_generated_node_manifest_abs_path__build}"  # File to write the behavior tree node plugin manifest to
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE _node_library_paths
        RESULT_VARIABLE _return_code
        ERROR_VARIABLE _error
    )
    if(NOT _return_code EQUAL 0)
        message(
            FATAL_ERROR
            "Failed to create node plugin manifest '${metadata_id}' (Return code: ${_return_code}). Manifest files: [${ARGS_UNPARSED_ARGUMENTS}]
Build info: [${_AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO}]
Output file: ${_generated_node_manifest_abs_path__build}
${_error}"
)
    endif()
    message(STATUS "Generated behavior tree node manifest ${_generated_node_manifest_abs_path__build} (Relevant libraries: [${_node_library_paths}]).")

    # Use the above created manifest for generating a node model
    set(_generated_node_model_abs_path__build "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${_generated_node_model_file_name}")
    add_custom_command(OUTPUT "${_generated_node_model_abs_path__build}"
        COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_CMD}"
            "\"${_generated_node_manifest_abs_path__build}\"" # Path to the generated node plugin manifest

            # Exhaustive list of libraries to be loaded by ClassLoader.
            # create_node_manifest previously collected all library paths that are required to successfully load the nodes specified in the manifest file.
            # We pass this variable to the command to specify which libraries to load and to add target-level dependencies for those targets mentioned in any $<TARGET_FILE:tgt> generator expressions.
            # Therefore, we configure the compilation so that any shared libraries created by the package invoking the macro are built before being used here.
            # If we wouldn't do this, there would be an error saying 'there is no rule to make target ...'.
            # Additionally, this variable needs to be passed to DEPENDS to create a file-level dependency to the shared library files which makes sure that the command is executed when they are recompiled.
            "\"${_node_library_paths}\""

            "\"${_generated_node_model_abs_path__build}\"" # File to write the behavior tree node model to
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        DEPENDS "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_CMD}" ${ARGS_NODE_MANIFEST} ${_node_library_paths}
        COMMENT "Generating behavior tree node model '${metadata_id}' with libraries [${_node_library_paths}] and manifest file ${_generated_node_manifest_abs_path__build}."
    )
    set(_generate_node_model_target "create_node_model__${_custom_target_suffix}")
    add_custom_target("${_generate_node_model_target}" ALL
        DEPENDS "${_generated_node_model_abs_path__build}"
    )

    # Install the generated node plugin manifest file
    install(
        FILES "${_generated_node_manifest_abs_path__build}"
        DESTINATION "${_generated_node_manifest_rel_dir__install}"
    )

    # Install the generated node model file
    install(
        FILES "${_generated_node_model_abs_path__build}"
        DESTINATION "${_generated_node_model_rel_dir__install}"
    )

    if(NOT "${ARGS_MODEL_HEADER_TARGET}" STREQUAL "")
        # Generate a header that makes the node metadata available to downstream C++ source code
        set(_generated_node_model_header_abs_path__build "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${PROJECT_NAME}/${metadata_id}.hpp")
        file(MAKE_DIRECTORY "${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}/${PROJECT_NAME}")
        add_custom_command(OUTPUT "${_generated_node_model_header_abs_path__build}"
            COMMAND "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_HEADER_CMD}"
                "\"${_generated_node_manifest_abs_path__build}\"" # Path to the generated node plugin manifest
                "\"${_generated_node_model_abs_path__build}\"" # Path to the generated node model
                "\"${PROJECT_NAME}\"" # Name of the package (defining the namespace to be used)
                "\"${_generated_node_model_header_abs_path__build}\"" # Header output file
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS "${_AUTO_APMS_BEHAVIOR_TREE_CORE__CREATE_NODE_MODEL_HEADER_CMD}" "${_generated_node_model_abs_path__build}"
            COMMENT "Generating behavior tree node model header '${metadata_id}' with manifest file ${_generated_node_manifest_abs_path__build} and model file ${_generated_node_model_abs_path__build}."
        )

        set(_generate_node_model_header_target "create_node_model_header__${_custom_target_suffix}")
        add_custom_target("${_generate_node_model_header_target}" ALL
            DEPENDS "${_generated_node_model_header_abs_path__build}"
        )
        add_dependencies("${_generate_node_model_header_target}" "${_generate_node_model_target}")
        add_dependencies("${ARGS_MODEL_HEADER_TARGET}" "${_generate_node_model_header_target}")

        # Make generated header includable to this and downstream targets
        get_target_property(_type "${ARGS_MODEL_HEADER_TARGET}" TYPE)
        if("${_type}" STREQUAL "INTERFACE_LIBRARY")
            set(_keyword "INTERFACE")
        else()
            set(_keyword "PUBLIC")
        endif()
        target_include_directories("${ARGS_MODEL_HEADER_TARGET}" "${_keyword}"
            $<BUILD_INTERFACE:${_AUTO_APMS_BEHAVIOR_TREE_CORE__BUILD_DIR_ABSOLUTE}>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )
        # IMPORTANT: The target MODEL_HEADER_TARGET should link against auto_apms_behavior_tree_core, because otherwise
        # if a downstream package is including the model header and doesn't link against auto_apms_behavior_tree_core,
        # CMake will fail to compile the source code. Therefore, it is also recommended to add auto_apms_behavior_tree_core
        # to ament_export_dependencies, since we don't want downstream packages to call find_package because of an
        # unresolved dependency of a target from another package.
        # We do not add ament_target_dependencies("${ARGS_MODEL_HEADER_TARGET}" auto_apms_behavior_tree_core)
        # here, because we cannot detect which signature of target_link_libraries (plain or all-keyword)
        # regarding the scope keywords (PRIVATE, PUBLIC or INTERFACE) has been used before.
        # CMake only allows to use one type of signature, so it's impossible here to add a call to ament_target_dependencies since it
        # calls target_link_libraries under the hood with the scope keywords we give it. Furthermore, we must
        target_links_against("${ARGS_MODEL_HEADER_TARGET}" "auto_apms_behavior_tree_core::auto_apms_behavior_tree_core" is_linked)
        if(NOT is_linked)
            message(WARNING "auto_apms_behavior_tree_generate_node_metadata(): The target '${ARGS_MODEL_HEADER_TARGET}' provided using the optional argument MODEL_HEADER_TARGET doesn't link against package 'auto_apms_behavior_tree_core'.
You should consider adding this dependency and export it to enable downstream packages including the behavior tree node model header generated by this macro:
ament_target_dependencies(${ARGS_MODEL_HEADER_TARGET}
    ... # Other packages
    auto_apms_behavior_tree_core
)
ament_export_dependencies(
    ... # Other packages
    auto_apms_behavior_tree_core
)
Otherwise, downstream packages are forced to add this dependency themselves since CMake will fail to compile any source code because it cannot resolve the #include statements inside the header."
)
        endif()

        # Install the generated node model header file
        install(
            FILES "${_generated_node_model_header_abs_path__build}"
            DESTINATION "include/${PROJECT_NAME}/${PROJECT_NAME}" # Using double project name as directory avoids collisions with existing files
        )
    endif()

    # Store the metadata information for reusing it during auto_apms_behavior_tree_declare_trees()
    list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_BUILD_INFO "${metadata_id}@${_generated_node_manifest_abs_path__build}")
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__NODE_MANIFEST "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__NODE_MANIFEST}${metadata_id}|${_generated_node_manifest_rel_dir__install}/${_generated_node_manifest_file_name}\n")

endmacro()