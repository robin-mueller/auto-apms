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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    with_orchestrator_launch_arg = DeclareLaunchArgument(
        "with_orchestrator",
        description="Whether to load the mission orchestrator together with the other components or not",
        default_value="true",
    )
    with_orchestrator = LaunchConfiguration("with_orchestrator")

    composable_node_descriptions = [
        ComposableNode(
            package="auto_apms_mission",
            plugin="auto_apms_mission::MissionExecutor",
            parameters=[
                {
                    "build_handler": "auto_apms_behavior_tree::TreeFromStringBuildHandler",
                    "allow_other_build_handlers": False,
                    "groot2_port": 5666,
                }
            ],
        ),
        ComposableNode(
            package="auto_apms_mission",
            plugin="auto_apms_mission::EventMonitorExecutor",
            parameters=[
                {
                    "build_handler": "auto_apms_behavior_tree::TreeFromStringBuildHandler",
                    "allow_other_build_handlers": False,
                    "groot2_port": 5777,
                }
            ],
        ),
        ComposableNode(
            package="auto_apms_mission",
            plugin="auto_apms_mission::EventHandlerExecutor",
            parameters=[
                {
                    "build_handler": "auto_apms_behavior_tree::TreeFromStringBuildHandler",
                    "allow_other_build_handlers": False,
                    "groot2_port": 5888,
                }
            ],
        ),
    ]

    composable_node_descriptions_with_orchestrator = composable_node_descriptions.copy()
    composable_node_descriptions_with_orchestrator.append(
        ComposableNode(
            package="auto_apms_mission",
            plugin="auto_apms_mission::OrchestratorExecutor",
            parameters=[
                {
                    "groot2_port": 5555,
                }
            ],
        ),
    )

    # Multithreading is required due to parallel spin_until_future_complete function calls
    return LaunchDescription(
        [
            with_orchestrator_launch_arg,
            ComposableNodeContainer(
                name="mission_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=composable_node_descriptions_with_orchestrator,
                output="screen",
                emulate_tty=True,
                condition=IfCondition(with_orchestrator),
            ),
            ComposableNodeContainer(
                name="mission_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=composable_node_descriptions,
                output="screen",
                emulate_tty=True,
                condition=UnlessCondition(with_orchestrator),
            ),
        ]
    )
