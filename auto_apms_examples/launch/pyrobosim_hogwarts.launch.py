#!/usr/bin/env python3

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

import json
import numpy as np

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node

GRYFFINDOR_NAMES = ["potter", "granger", "weasley"]
SLYTHERIN_NAMES = ["malfoy", "crabbe", "goyle"]


def create_nodes(context: LaunchContext):
    max_student_num = len(GRYFFINDOR_NAMES) + len(SLYTHERIN_NAMES)
    student_num = int(context.launch_configurations["student_num"])
    if student_num > max_student_num:
        raise ValueError(f"Argument 'student_num' must not be bigger than {max_student_num}.")

    gryffindor_num = int(np.floor(student_num / 2))
    slytherin_num = int(np.floor(student_num / 2))
    if gryffindor_num + slytherin_num < student_num:
        if np.random.choice(2) > 0:
            gryffindor_num += 1
        else:
            slytherin_num += 1

    gryffindor_names = np.random.choice(GRYFFINDOR_NAMES, gryffindor_num, replace=False).tolist()
    slytherin_names = np.random.choice(SLYTHERIN_NAMES, slytherin_num, replace=False).tolist()
    return (
        [
            Node(
                package="auto_apms_simulation",
                executable="world.py",
                arguments=[
                    "hogwarts",
                    json.dumps(
                        {
                            "size": 4.0,
                            "hallway_num": int(context.launch_configurations["hallway_num"]),
                            "gryffindor_names": gryffindor_names,
                            "slytherin_names": slytherin_names,
                        }
                    ),
                ],
            ),
        ]
        + [
            Node(
                package="auto_apms_behavior_tree",
                executable="tree_executor",
                name=student,
                arguments=["auto_apms_examples::hogwarts::GryffindorTree"],
                parameters=[{"bb.student": student}],
            )
            for student in gryffindor_names
        ]
        + [
            Node(
                package="auto_apms_behavior_tree",
                executable="tree_executor",
                name=student,
                arguments=["auto_apms_examples::hogwarts::SlytherinTree"],
                parameters=[{"bb.student": student}],
            )
            for student in slytherin_names
        ]
    )


def generate_launch_description():
    hallway_num_arg = DeclareLaunchArgument(
        "hallway_num", description="Number of magical hallways in Hogwarts.", default_value="4"
    )
    student_num_arg = DeclareLaunchArgument(
        "student_num", description="Number of wizardry students in Hogwarts.", default_value="2"
    )
    return LaunchDescription(
        [
            hallway_num_arg,
            student_num_arg,
            OpaqueFunction(function=create_nodes),
        ]
    )
