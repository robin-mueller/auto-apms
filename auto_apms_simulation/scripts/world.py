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

import rclpy
import threading
import json

from argparse import ArgumentParser
from auto_apms_simulation.example import create_world_from_name, get_world_names
from pyrobosim.gui import start_gui
from pyrobosim_ros.ros_interface import WorldROSWrapper


def main():
    parser = ArgumentParser(description="Create a 2D simulation world.", allow_abbrev=False)
    parser.add_argument(
        "name", nargs="?", help="Name of the world to create.", default="hogwarts", choices=get_world_names()
    )
    parser.add_argument(
        "options",
        nargs="?",
        help="Options to pass to the world during initialization (Use json format).",
        default="{}",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WorldROSWrapper(
        create_world_from_name(args.name, **json.loads(args.options)), state_pub_rate=0.1, dynamics_rate=0.01
    )
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True, auto_spin=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)


if __name__ == "__main__":
    main()
