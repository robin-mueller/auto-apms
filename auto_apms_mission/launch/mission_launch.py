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

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def spawn_orchestrator_node(context: LaunchContext):
    return [
        Node(
            package="auto_apms_behavior_tree",
            executable="run_tree",
            name=context.launch_configurations["orchestrator_name"],
            parameters=[
                {
                    "build_handler": PythonExpression(
                        [
                            "'auto_apms_mission::SingleNodeMissionBuildHandler' if bool('",
                            context.launch_configurations["use_multiple_nodes"],
                            "') else 'auto_apms_mission::MultipleNodesMissionBuildHandler'",
                        ]
                    ),
                    "allow_other_build_handlers": False,
                    "groot2_port": -1,
                },
                json.loads(context.launch_configurations["orchestrator_params"]),
            ],
            arguments=[LaunchConfiguration("config")],
            output="screen",
            emulate_tty=True,
        ),
    ]


def generate_launch_description():
    config_launch_arg = DeclareLaunchArgument(
        "config", description="Resource identity for the mission configuration file."
    )
    orchestrator_name_arg = DeclareLaunchArgument(
        "orchestrator_name",
        default_value="orchestrator",
        description="Name of the mission orchestrator node.",
    )
    orchestrator_params_arg = DeclareLaunchArgument(
        "orchestrator_params",
        default_value="{}",
        description="JSON encoded dictionary that is used as parameter overrides for the orchestrator node.",
    )
    use_multiple_nodes_arg = DeclareLaunchArgument(
        "use_multiple_nodes",
        default_value="false",
        description="Delegate mission execution as well as event monitoring and handling to individual nodes.",
    )

    return LaunchDescription(
        [
            config_launch_arg,
            orchestrator_name_arg,
            orchestrator_params_arg,
            use_multiple_nodes_arg,
            ComposableNodeContainer(
                name="mission_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
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
                ],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(LaunchConfiguration("use_multiple_nodes")),
            ),
            OpaqueFunction(function=spawn_orchestrator_node),
        ]
    )
