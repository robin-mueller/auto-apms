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

macro(auto_apms_behavior_tree_generate_node_metadata metadata_id)

    # Parse arguments
    set(options "")
    set(oneValueArgs GENERATED_MANIFEST_FILE_NAME GENERATED_MODEL_FILE_NAME)
    set(multiValueArgs "")
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(${ARGS_UNPARSED_ARGUMENTS} STREQUAL "")
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

    if(NOT ${ARGS_GENERATED_MANIFEST_FILE_NAME} STREQUAL "")
        get_filename_component(_generated_node_manifest_file_stem "${ARGS_GENERATED_MANIFEST_FILE_NAME}" NAME_WE)
        set(_generated_node_manifest_file_name "${_generated_node_manifest_file_stem}.yaml")
    endif()

    if(NOT ${ARGS_GENERATED_MODEL_FILE_NAME} STREQUAL "")
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
            "Failed to create node plugin manifest '${metadata_id}' (Return code: ${_return_code}). Manifest files: [${ARGS_UNPARSED_ARGUMENTS}]\nBuild info: [${_AUTO_APMS_BEHAVIOR_TREE__NODE_BUILD_INFO}]\nOutput file: ${_generated_node_manifest_abs_path__build}\n${_error}"
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
            # We pass this variable to the command to specifiy which libraries to load and to add target-level dependencies for those targets mentioned in any $<TARGET_FILE:tgt> generator expressions.
            # Therefore, we configure the compilation so that any shared libraries created by the package invoking the macro are built before being used here.
            # If we wouldn't do this, there would be an error saying 'there is no rule to make target ...'.
            # Additionally, this variable needs to be passed to DEPENDS to create a file-level dependency to the shared library files which makes sure that the command is executed when they are recompiled.
            "\"${_node_library_paths}\""

            "\"${_generated_node_model_abs_path__build}\"" # File to write the behavior tree node model to
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        DEPENDS ${ARGS_NODE_MANIFEST} ${_node_library_paths}
        COMMENT "Generating behavior tree node model '${metadata_id}' with libraries [${_node_library_paths}] and manifest file ${_generated_node_manifest_abs_path__build}.")
    add_custom_target(create_node_model__${_custom_target_suffix} ALL
        DEPENDS "${_generated_node_model_abs_path__build}")

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

    # Store the metadata information for reusing it during auto_apms_behavior_tree_declare_trees()
    list(APPEND _AUTO_APMS_BEHAVIOR_TREE__NODE_MANIFEST_BUILD_INFO "${metadata_id}@${_generated_node_manifest_abs_path__build}")
    set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__NODE_MANIFEST "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__NODE_MANIFEST}${metadata_id}|${_generated_node_manifest_rel_dir__install}/${_generated_node_manifest_file_name}\n")

endmacro()