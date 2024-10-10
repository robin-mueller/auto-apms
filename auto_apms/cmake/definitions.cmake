# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# ------------ AutoAPMS CMake Variable Definitions --------------
#

set(_AUTO_APMS_INTERNAL_CLI_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/internal")

set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_NAMES "names")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_CLASS "class_name")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_PACKAGE "package")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_LIBRARY "library")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_PORT "port")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_WAIT_TIMEOUT "wait_timeout")
set(_AUTO_APMS_BEHAVIOR_TREE__NODE_PLUGIN_MANIFEST_PARAM_REQUEST_TIMEOUT "request_timeout")

set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__TREE "behavior_trees")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_DIR_NAME__NODE "behavior_tree_nodes")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__TREE "auto_apms_behavior_tree__tree")
set(_AUTO_APMS_BEHAVIOR_TREE__RESOURCE_TYPE_NAME__NODE "auto_apms_behavior_tree__node")