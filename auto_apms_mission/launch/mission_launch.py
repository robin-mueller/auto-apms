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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config_launch_arg = DeclareLaunchArgument(
        "config", description="Resource identity for the mission configuration file"
    )
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            config_launch_arg,
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("auto_apms_mission"), "launch", "mission_components_launch.py"]
                    )
                ),
                launch_arguments={"with_orchestrator": "false"}.items(),
                condition=IfCondition(
                    "false"
                ),  # TODO: Implement outsourcing of event monitor and mission executor to different nodes
            ),
            Node(
                executable="orchestrator",
                package="auto_apms_mission",
                parameters=[
                    {
                        "groot2_port": 5555,
                    }
                ],
                arguments=[config],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
