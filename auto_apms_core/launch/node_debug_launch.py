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
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = LaunchConfiguration("package")
    package_arg = DeclareLaunchArgument("package", description="Name of the package registering the executable")
    executable = LaunchConfiguration("executable")
    executable_arg = DeclareLaunchArgument("executable", description="Name of the executable to run in debug mode")

    return LaunchDescription(
        [
            package_arg,
            executable_arg,
            Node(
                namespace="",
                package=package,
                executable=executable,
                prefix=["gdbserver localhost:3000"],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
