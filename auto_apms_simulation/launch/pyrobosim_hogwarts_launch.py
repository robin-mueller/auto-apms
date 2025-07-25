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
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction,
    UnregisterEventHandler,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessIO
from launch.conditions import IfCondition
from launch.substitutions import (
    PythonExpression,
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

GRYFFINDOR_NAMES = ["potter", "granger", "weasley"]
SLYTHERIN_NAMES = ["malfoy", "crabbe", "goyle"]

TICK_RATE_SEC = 0.1


def spawn_nodes(context: LaunchContext):
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
    name_house_tuples = [(name, "gryffindor") for name in gryffindor_names] + [
        (name, "slytherin") for name in slytherin_names
    ]

    encoded_world_kwargs = json.dumps(
        {
            "size": 4.0,
            "hallway_num": int(context.launch_configurations["hallway_num"]),
            "gryffindor_names": gryffindor_names,
            "slytherin_names": slytherin_names,
        }
    )
    world_node = Node(
        package="auto_apms_simulation",
        executable="world.py",
        name="hogwarts_world_node",
        ros_arguments=["--log-level", "hogwarts_world_node:=WARN"],
        arguments=["hogwarts", encoded_world_kwargs],
        output="screen",
        emulate_tty=True,
    )

    start_tree_nodes_event_handler = OnProcessIO(
        target_action=world_node,
        on_stdout=lambda event: [
            GroupAction(
                [
                    Node(
                        package="auto_apms_behavior_tree",
                        executable="run_behavior",
                        name=student,
                        arguments=[
                            (
                                "auto_apms_simulation::hogwarts::GryffindorTree"
                                if house == "gryffindor"
                                else "auto_apms_simulation::hogwarts::SlytherinTree"
                            )
                        ],
                        parameters=[{"tick_rate": TICK_RATE_SEC, "bb.student": student}],
                        output="screen",
                        emulate_tty=True,
                    )
                    for student, house in name_house_tuples
                ],
                condition=IfCondition(
                    AndSubstitution(
                        PythonExpression([f"'{event.text.decode().strip()}' == 'WORLD_READY'"]),
                        NotSubstitution(LaunchConfiguration("mission")),
                    )
                ),
            ),
            GroupAction(
                [
                    IncludeLaunchDescription(
                        launch_description_source=PythonLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("auto_apms_mission"), "launch", "mission_launch.py"])
                        ),
                        launch_arguments={
                            "config": "auto_apms_simulation::pyrobosim_hogwarts_mission",
                            "orchestrator_name": student,
                            "orchestrator_params": json.dumps(
                                {"tick_rate": TICK_RATE_SEC, "bb.student": student, "bb.house": house}
                            ),
                            "multi_node": "false",
                        }.items(),
                    )
                    for student, house in name_house_tuples
                ],
                condition=IfCondition(
                    AndSubstitution(
                        PythonExpression([f"'{event.text.decode().strip()}' == 'WORLD_READY'"]),
                        LaunchConfiguration("mission"),
                    )
                ),
            ),
            UnregisterEventHandler(
                start_tree_nodes_event_handler,
                condition=IfCondition(PythonExpression([f"'{event.text.decode().strip()}' == 'WORLD_READY'"])),
            ),
        ],
    )

    return [
        world_node,
        RegisterEventHandler(start_tree_nodes_event_handler),
    ]


def generate_launch_description():
    hallway_num_arg = DeclareLaunchArgument(
        "hallway_num", description="Number of magical hallways in Hogwarts.", default_value="3"
    )
    student_num_arg = DeclareLaunchArgument(
        "student_num", description="Number of wizardry students in Hogwarts.", default_value="4"
    )
    mission_arg = DeclareLaunchArgument(
        "mission", description="Start the Hogwarts mission orchestrator.", default_value="false"
    )

    return LaunchDescription(
        [
            hallway_num_arg,
            student_num_arg,
            mission_arg,
            OpaqueFunction(function=spawn_nodes),
        ]
    )
