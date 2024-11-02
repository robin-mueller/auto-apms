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

if(NOT DEFINED _AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE)
    # Make sure this also works when included manually in the original package
    set(_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE "share/${PROJECT_NAME}/${PROJECT_NAME}")
endif()

if(DEFINED _AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_XML_CONTENT)
    file(GENERATE
        OUTPUT "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${PROJECT_NAME}_node_plugins.xml"
        CONTENT "<class_libraries>\n${_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_XML_CONTENT}</class_libraries>"
    )
    install(
        FILES
        "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${PROJECT_NAME}_node_plugins.xml"
        DESTINATION
        "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}"
    )
    ament_index_register_resource(
        "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE}"
        CONTENT
        "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}/${PROJECT_NAME}_node_plugins.xml"
    )
endif()

if(DEFINED _AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE)
    ament_index_register_resource(
        "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE}"
        CONTENT
        "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_FILE__TREE}"
    )
endif()

if(DEFINED _AUTO_APMS_BEHAVIOR_TREE__BUILDER_PLUGIN_XML_CONTENT)
    file(GENERATE
        OUTPUT "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${PROJECT_NAME}_builder_plugins.xml"
        CONTENT "<class_libraries>\n${_AUTO_APMS_BEHAVIOR_TREE__BUILDER_PLUGIN_XML_CONTENT}</class_libraries>"
    )
    install(
        FILES
        "${PROJECT_BINARY_DIR}/${_AUTO_APMS_BEHAVIOR_TREE_BUILD_DIR_RELATIVE}/${PROJECT_NAME}_builder_plugins.xml"
        DESTINATION
        "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}"
    )
    ament_index_register_resource(
        "${_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__BUILDER}"
        CONTENT
        "${_AUTO_APMS_BEHAVIOR_TREE_RESOURCES_DIR_RELATIVE}/${PROJECT_NAME}_builder_plugins.xml"
    )
endif()

