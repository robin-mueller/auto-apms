from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace="",
                package="px4_behavior",
                executable="node_debug_exe",
                prefix=["gdbserver localhost:3000"],
                output="screen",
                emulate_tty=True,
                parameters=[{"state_change_logging": True}]
            )
        ]
    )
