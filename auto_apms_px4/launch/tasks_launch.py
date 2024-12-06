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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ALL_TASK_NAMES = [
    "auto_apms_px4::ArmDisarmTask",
    "auto_apms_px4::EnableHoldTask",
    "auto_apms_px4::GoToTask",
    "auto_apms_px4::LandTask",
    "auto_apms_px4::TakeoffTask",
    "auto_apms_px4::RTLTask",
    "auto_apms_px4::MissionTask",
]


def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="task_container_node",
                namespace="",
                exec_name="task_container",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(package="auto_apms_px4", plugin=name) for name in ALL_TASK_NAMES
                ],
                output="screen",
                emulate_tty=True,
            )
        ]
    )
