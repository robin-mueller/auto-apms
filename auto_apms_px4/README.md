# auto_apms_px4

To successfully build this package, you must manually install the following dependencies (they are not available with rosdep):

| Name | Description |
| :---| :--- |
| [px4_ros2_cpp](https://github.com/Auterion/px4-ros2-interface-lib) | Library that allows to model PX4 flight modes as ROS 2 applications. |
| [px4_msgs](https://github.com/PX4/px4_msgs) | ROS 2 message definitions for the PX4 Autopilot project. |

You should download the source code of these packages using `vcs`:

```bash
# Make sure you are inside your workspace's root directory
vcs import < "src/auto-apms/dependencies.repos" "src" --recursive
```
