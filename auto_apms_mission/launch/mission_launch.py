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

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", description="Namespace for the task nodes", default_value=""
    )
    namespace = LaunchConfiguration("namespace")

    # Multithreading is required due to parallel spin_until_future_complete function calls
    container = ComposableNodeContainer(
        name="mission_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="auto_apms_mission",
                namespace=namespace,
                plugin="auto_apms_mission::MissionOrchestrator",
                parameters=[
                    {
                        "build_handler": "auto_apms_mission::MissionBuildHandler",
                        "allow_other_build_handlers": False,
                        "groot2_port": 5555,
                    }
                ],
            ),
            ComposableNode(
                name="safety_monitor",
                namespace=namespace,
                package="auto_apms_behavior_tree",
                plugin="auto_apms_behavior_tree::TreeExecutorNode",
                parameters=[{"groot2_port": 5666}],
            ),
            ComposableNode(
                name="contingency_handler",
                namespace=namespace,
                package="auto_apms_behavior_tree",
                plugin="auto_apms_behavior_tree::TreeExecutorNode",
                parameters=[{"groot2_port": 5777}],
            ),
            ComposableNode(
                name="nominal_mission",
                namespace=namespace,
                package="auto_apms_behavior_tree",
                plugin="auto_apms_behavior_tree::TreeExecutorNode",
                parameters=[{"groot2_port": 5888}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([namespace_launch_arg, container])
