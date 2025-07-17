# Copyright 2025 Robin Müller
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

from auto_apms_util.resources import *
from auto_apms_behavior_tree_core.resources import *

BASE_CLASS_TYPE__BEHAVIOR_TREE_BUILD_HANDLER = "auto_apms_behavior_tree::TreeBuildHandlerFactoryInterface"


def get_behavior_build_handler_plugins(exclude_packages: set[str] = None) -> list[str]:
    """
    Get all behavior tree build handler plugin names.

    This is a convenience function that finds all plugins with the behavior tree build handler
    base class type.

    Args:
        exclude_packages: Packages to exclude when searching for plugins.

    Returns:
        List of all behavior tree build handler plugin names.

    Raises:
        ResourceError: If failed to find or parse plugin manifest files.
    """
    return get_plugin_names_with_base_type(BASE_CLASS_TYPE__BEHAVIOR_TREE_BUILD_HANDLER, exclude_packages)
