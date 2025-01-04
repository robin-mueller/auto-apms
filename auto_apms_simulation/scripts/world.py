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
import time
import rclpy
import sys
import signal
import threading

from rclpy.executors import MultiThreadedExecutor
from argparse import ArgumentParser
from pyrobosim.core import World
from pyrobosim.gui import PyRoboSimGUI
from pyrobosim_ros.ros_interface import WorldROSWrapper
from auto_apms_simulation.example import create_world_from_name, get_world_names


def main():
    parser = ArgumentParser(description="Create a 2D simulation world.", allow_abbrev=False)
    parser.add_argument(
        "name", nargs="?", help="Name of the world to create.", default="hogwarts", choices=get_world_names()
    )
    parser.add_argument(
        "options",
        nargs="?",
        help="Options dictionary to pass to the world during initialization (Use json format).",
        default="{}",
    )

    # Create the world
    args, unknown_args = parser.parse_known_args()
    world: World = create_world_from_name(args.name, **json.loads(args.options))

    # Initialize the node
    rclpy.init(args=unknown_args)
    node = WorldROSWrapper(
        world,
        state_pub_rate=0.25,
        dynamics_rate=0.1,
        dynamics_latch_time=0.5,
        dynamics_ramp_down_time=0.5,
        dynamics_enable_collisions=False,
    )
    node.start(wait_for_gui=False, auto_spin=False)

    def spin():
        while not world.has_gui:
            node.get_logger().info("Waiting for GUI...")
            time.sleep(1.0)
        print("WORLD_READY", flush=True)  # Allow other processes to react when world node is ready
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    # Spin the node in a separate thread
    node_thread = threading.Thread(target=spin, name=node.get_name(), daemon=True)
    node_thread.start()

    # Start GUI in main thread
    app = PyRoboSimGUI(world, unknown_args, True)

    def shutdown_world(*args):
        node.get_logger().info("SIGINT detected. Allowing clients to shut down...")
        time.sleep(1.0)  # Sleep for a short while to allow clients to shut down before the servers are destroyed
        node.get_logger().info("Quitting the application.")
        app.quit()

    signal.signal(signal.SIGINT, shutdown_world)

    # Run the event loop of the Qt application
    code = app.exec()

    # Shut down the node and exit the program
    node.get_logger().info("World node shutdown.")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    node_thread.join()
    sys.exit(code)


if __name__ == "__main__":
    main()
