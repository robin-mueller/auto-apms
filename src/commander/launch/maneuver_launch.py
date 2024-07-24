from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        "namespace", description="namespace for the maneuver nodes", default_value=""
    )

    namespace = LaunchConfiguration("namespace")

    container = ComposableNodeContainer(
        name="maneuver_container_node",
        exec_name="maneuver_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::ArmDisarmManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::EnableHoldManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::GoToManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::LandManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::RTLManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::TakeoffManeuver",
            ),
            ComposableNode(
                package="commander",
                namespace=namespace,
                plugin="commander::MissionManeuver",
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([namespace_launch_arg, container])
