from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                namespace="",
                package="contingency_manager",
                executable="node_debug_exe",
                prefix=["gdbserver localhost:3000"],
                output="screen",
                emulate_tty=True,
                parameters=[{"state_change_logging": True}]
            )
        ]
    )
