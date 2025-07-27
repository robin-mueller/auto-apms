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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from auto_apms_behavior_tree.launch import RunBehavior, BehaviorResource


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("approach", choices=["graphical", "programmatic"]),
            # Spawn the simple skill server
            Node(package="auto_apms_examples", executable="simple_skill_server"),
            # Spawn the behavior executor for the simple skill tree
            RunBehavior(
                build_request=BehaviorResource("auto_apms_examples::simple_skill_tree::SimpleSkillDemo"),
                blackboard={"msg": "Custom message", "n_times": 10},
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration("approach"), "graphical")),
            ),
            RunBehavior(
                build_handler="auto_apms_examples::SimpleSkillBuildHandler",
                blackboard={"msg": "Custom message", "n_times": 10},
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration("approach"), "programmatic")),
            ),
        ]
    )
