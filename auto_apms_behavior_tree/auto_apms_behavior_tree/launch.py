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

from typing import Optional, Any, Text
from launch.frontend import expose_action
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch_ros.actions import Node
from launch_ros.parameters_type import Mapping, SomeParameterName, Any, SomeParameters, SomeParametersDict
from auto_apms_behavior_tree_core.resources import BehaviorResource, NodeManifest


class SubstitutionWithPrefix(Substitution):
    def __init__(self, substitution: SomeSubstitutionsType, prefix: Text) -> None:
        self.__substitution = substitution
        self.__prefix = prefix

    def __repr__(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"SubstitutionWithPrefix('{self.__prefix}' + {self.__substitution.describe()})"

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution."""
        return self.__prefix + perform_substitutions(context, normalize_to_list_of_substitutions(self.__substitution))


@expose_action("run_behavior")
class RunBehavior(Node):
    """Action to launch a behavior executor and run a behavior on the host machine."""

    def __init__(
        self,
        *,
        build_request: Optional[SomeSubstitutionsType | BehaviorResource] = None,
        entrypoint: Optional[SomeSubstitutionsType] = None,
        node_manifest: Optional[SomeSubstitutionsType | NodeManifest] = None,
        build_handler: Optional[SomeSubstitutionsType] = None,
        blackboard: Optional[Mapping[SomeParameterName, Any]] = None,
        scripting_enums: Optional[Mapping[SomeParameterName, int]] = None,
        **kwargs,
    ):
        """
        Configure the executor and specify the behavior to run.

        Other arguments are forwarded to the Node action.
        """
        if isinstance(build_request, BehaviorResource):
            build_request_sub = build_request.build_request
            if entrypoint is None:
                entrypoint = build_request.entrypoint
            if node_manifest is None:
                node_manifest = build_request.node_manifest
            if build_handler is None:
                build_handler = build_request.default_build_handler
        else:
            build_request_sub = build_request

        node_manifest_sub = node_manifest.dump() if isinstance(node_manifest, NodeManifest) else node_manifest

        parameters: SomeParameters = kwargs.pop("parameters", [])
        custom_params: SomeParametersDict = {}

        if build_handler:
            custom_params.update({"build_handler": build_handler})

        custom_params.update({SubstitutionWithPrefix(name, "bb."): value for name, value in (blackboard or {}).items()})
        custom_params.update(
            {SubstitutionWithPrefix(name, "enum."): value for name, value in (scripting_enums or {}).items()}
        )

        # Build arguments list, filtering out None values
        arguments = []
        arguments.append(build_request_sub if build_request_sub is not None else "")
        arguments.append(entrypoint if entrypoint is not None else "")
        arguments.append(node_manifest_sub if node_manifest_sub is not None else "")

        super().__init__(
            package="auto_apms_behavior_tree",
            executable="run_behavior",
            arguments=arguments,
            parameters=list(parameters) + [custom_params],
            **kwargs,
        )

    @classmethod
    def parse(cls, entity, parser):
        return super().parse(entity, parser)
