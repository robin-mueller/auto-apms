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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

ALL_TASK_NAMES = [
    "px4_behavior::ArmDisarmTask",
    "px4_behavior::EnableHoldTask",
    "px4_behavior::GoToTask",
    "px4_behavior::LandTask",
    "px4_behavior::TakeoffTask",
    "px4_behavior::RTLTask",
    "px4_behavior::MissionTask",
]


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", description="namespace for the task nodes", default_value=""
    )

    namespace = LaunchConfiguration("namespace")

    composable_nodes = [
        ComposableNode(
            package="px4_behavior",
            namespace=namespace,
            plugin=name,
        )
        for name in ALL_TASK_NAMES
    ]

    container = ComposableNodeContainer(
        name="task_container_node",
        exec_name="task_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([namespace_launch_arg, container])
