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
# Add behavior tree build handler plugins to the package's resources.
#
# This macro must be called to make behavior tree build handlers available
# at runtime. They may be loaded using TreeBuildHandlerLoader (a subclass of
# pluginlib::ClassLoader).
#
# :param target: Shared library target implementing the behavior tree
#   build handlers registered under ARGN.
# :type target: string
# :param ARGN: The unique names of build handler classes being registered with this
#   macro call and exported by the shared library target.
# :type ARGN: list of strings
#
# @public
#
macro(auto_apms_behavior_tree_register_build_handlers target)
  auto_apms_util_register_plugins(
    ${target}
    "auto_apms_behavior_tree::TreeBuildHandlerFactoryInterface"
    ${ARGN}
    FACTORY_TEMPLATE_CLASS "auto_apms_behavior_tree::TreeBuildHandlerFactoryTemplate"
  )
endmacro()
