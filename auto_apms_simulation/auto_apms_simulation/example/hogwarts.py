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

import numpy as np

from pyrobosim.core import World, Room, Robot, Hallway, Pose, Location, Object
from pyrobosim.navigation import ConstantVelocityExecutor, RRTPlanner

from ..util import create_square_from_center


class HogwartsWorld(World):
    NAME = "hogwarts"

    def __init__(
        self,
        size: float = 4.0,
        hallway_num: int = 4,
        gryffindor_names: list[str] = ["potter"],
        slytherin_names: list[str] = ["malfoy"],
    ):
        super().__init__(self.NAME)

        wall_width = size / 20
        hallway_width = size / (hallway_num * (1 + 2 * wall_width) - 2 * wall_width)
        student_radius = hallway_width / 2.5
        boss_size = student_radius * 2.0

        Location.set_metadata(None)
        Location.metadata.data = {
            "boss": {
                "footprint": {"type": "box", "dims": [boss_size, boss_size], "height": 0.0},
                "nav_poses": [
                    {"position": {"x": 0.0, "y": boss_size}, "rotation_eul": {"yaw": -np.pi / 2}},  # Above
                    {"position": {"x": 0.0, "y": -boss_size}, "rotation_eul": {"yaw": np.pi / 2}},  # Below
                ],
                "locations": [{"name": "hand", "footprint": {"type": "parent", "padding": boss_size / 10.0}}],
            }
        }

        Object.set_metadata(None)
        Object.metadata.data = {
            "magical_item": {"footprint": {"type": "circle", "radius": boss_size / 10.0}, "height": 0.0},
        }

        center_y: float = 0.0
        center0 = (-size, center_y)
        center1 = (size, center_y)

        gryffindor = Room(
            "gryffindor",
            create_square_from_center(*center0, size),
            color="#ae0001",
            nav_poses=[Pose(*center0, z=0.0, yaw=0.0)],
            wall_width=wall_width,
        )
        slytherin = Room(
            "slytherin",
            create_square_from_center(*center1, size),
            color="#2a623d",
            nav_poses=[Pose(*center1, z=0.0, yaw=np.pi)],
            wall_width=wall_width,
        )
        self.add_room(room=gryffindor)
        self.add_room(room=slytherin)

        for i, y_point in enumerate(
            np.linspace(center_y + size / 2 - hallway_width / 2, center_y - size / 2 + hallway_width / 2, hallway_num)
        ):
            self.add_hallway(
                hallway=Hallway(
                    gryffindor,
                    slytherin,
                    f"hallway{i}",
                    width=hallway_width,
                    wall_width=wall_width,
                    conn_method="points",
                    conn_points=[
                        center0,
                        (center0[0] + size / 2, y_point),
                        (center1[0] - size / 2, y_point),
                        center1,
                    ],
                )
            )

        dumbledore = Location(
            "dumbledore",
            parent=gryffindor,
            category="boss",
            pose=Pose(center0[0] - size / 2 + boss_size, center0[1], 0.0),
            color=gryffindor.viz_color,
        )
        voldemort = Location(
            "voldemort",
            parent=slytherin,
            category="boss",
            pose=Pose(center1[0] + size / 2 - boss_size, center1[1], 0.0),
            color=slytherin.viz_color,
        )
        self.add_location(location=dumbledore)
        self.add_location(location=voldemort)

        item_names = [
            "toms_diary",
            "ravenclaws_diadem",
            "slytherins_locket",
            "hufflepuffs_cup",
            "elder_wand",
            "resurrection_stone",
            "invisibility_cloak",
        ]
        for name in item_names:
            choices = [dumbledore, voldemort]
            boss_choice = np.random.choice(choices)
            if self.add_object(name=name, category="magical_item", parent=boss_choice) is None:
                choices.remove(boss_choice)
                self.add_object(name=name, category="magical_item", parent=choices[0])

        for names, room in [(gryffindor_names, gryffindor), (slytherin_names, slytherin)]:
            for i, name in enumerate(names, 1):
                sign = (
                    1 if room.centroid[0] - (abs(slytherin.centroid[0]) - abs(gryffindor.centroid[0])) / 2 <= 0 else -1
                )
                x = room.centroid[0] + sign * size / 2 - sign * student_radius * 2
                y = room.centroid[1] - size / 2 + i * size / (len(names) + 1)
                yaw = 0.0 if sign > 0 else np.pi
                self.add_robot(
                    Robot(
                        name,
                        radius=student_radius,
                        height=0,
                        path_planner=RRTPlanner(
                            world=self,
                            bidirectional=True,
                            rrt_connect=False,
                            rrt_star=True,
                            collision_check_step_dist=size / 12,
                            max_connection_dist=size / 6,
                            rewire_radius=1,
                            compress_path=True,
                        ),
                        path_executor=ConstantVelocityExecutor(validation_step_dist=size / 12),
                        color=room.viz_color,
                    ),
                    loc=room,
                    pose=Pose(x=x, y=y, yaw=yaw),
                )
