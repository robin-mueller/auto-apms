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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def spawn_orchestrator_node(context: LaunchContext):
    return [
        Node(
            package="auto_apms_behavior_tree",
            executable="run_tree",
            name="orchestrator",
            parameters=[
                {
                    "build_handler": (
                        "auto_apms_mission::SingleNodeMissionBuildHandler"
                        if context.launch_configurations["use_multiple_nodes"] == "false"
                        else "auto_apms_mission::MultipleNodesMissionBuildHandler"
                    ),
                    "allow_other_build_handlers": False,
                    "groot2_port": 5555,
                    # "state_change_logger": True
                }
            ],
            arguments=[context.launch_configurations["config"]],
            output="screen",
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    config_launch_arg = DeclareLaunchArgument(
        "config", description="Resource identity for the mission configuration file"
    )
    use_multiple_nodes_arg = DeclareLaunchArgument(
        "use_multiple_nodes",
        default_value="false",
        description="Delegate mission execution as well as event monitoring and handling to individual nodes",
    )

    return LaunchDescription(
        [
            config_launch_arg,
            use_multiple_nodes_arg,
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("auto_apms_mission"), "launch", "mission_components_launch.py"]
                    )
                ),
                launch_arguments={"with_orchestrator": "false"}.items(),
                condition=IfCondition(LaunchConfiguration("use_multiple_nodes")),
            ),
            OpaqueFunction(function=spawn_orchestrator_node),
        ]
    )
