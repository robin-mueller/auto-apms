<p align="center" width="100%">
    <img width="70%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>

<div align="center">

<h3>Automated Action Planning and Management System</h3>

<a href="https://robin-mueller.github.io/auto-apms-guide/introduction/about">![Docs](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=🎓Documentation)</a>
<a href="https://doi.org/10.5281/zenodo.14790307">![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14790307.svg)</a>
<a href="https://github.com/robin-mueller/auto-apms/releases">![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)</a>
<a href="https://github.com/robin-mueller/auto-apms/actions/workflows/test.yaml">![ROS 2](https://github.com/robin-mueller/auto-apms/actions/workflows/test.yaml/badge.svg)</a>

</div>

# ✨ Streamlining Behaviors in ROS 2

![auto-apms-gif](https://github.com/user-attachments/assets/0039aa09-9448-4102-9eb3-38138a805728)

**Start leveraging the advantages of Behavior Trees 🌳 fully integrated with ROS 2 🤖**

AutoAPMS is a **heavily extensible** development framework for **behavior-based ROS 2 applications**. It provides a **highly modular integration of behavior trees**, implements a **powerful execution engine** and offers convenient CLI tooling for deploying behaviors with ease.

The intention of this project is to make it significantly more user-friendly and less error prone to develop autonomous robotics with behavior trees. The core packages are written in C++ and a supplementary Python API exposes high-level features for scripting.

> [!NOTE]
> Currently we support **Linux only**! For a list of supported ROS 2 distros see the [GitHub build workflow](.github/workflows/test.yaml).

## Highlights

There are plenty of ROS 2 packages which provide an implementation for behavior trees. AutoAPMS adopts [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP), the most popular one when it comes to C++, and embeds it into the ROS 2 ecosystem so that developers have a much easier time writing custom behaviors and distributing them among robots. Here are some of the most prominent features offered by this repository:

- Efficient resource management using `ament_cmake` and `ament_index`

- Inherently extensible due to plugin-based design

- Flexible and highly configurable behavior execution engine

- Powerful C++ behavior tree builder API (a supplement to BehaviorTree.CPP)

- Easy integration of custom behavior tree node implementations

- Support for custom behavior definitions and tree builder algorithms

- `ros2cli` extensions for orchestrating the system from the command line

- Abstractions for PX4 Autopilot available with [auto-apms-px4](https://github.com/robin-mueller/auto-apms-px4)

# 🚀 Getting Started

The following installation guide helps you getting started with AutoAPMS.

1. Create a [ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository

    ```bash
    mkdir ros2_ws && cd ros2_ws
    (mkdir src && cd src && git clone https://github.com/robin-mueller/auto-apms.git)
    ```

1. Install all required dependencies. We assume that you already installed ROS 2 on your system

    ```bash
    rosdep init  # Skip this if rosdep has already been initialized
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

1. Build and install all packages required for `auto_apms_examples`

    ```bash
    colcon build --packages-up-to auto_apms_examples --symlink-install
    ```

1. Run your first behavior using `ros2 behavior`. This is an extension of the ROS 2 CLI introduced by the `auto_apms_ros2behavior` package

    ```bash
    source install/setup.bash
    ros2 behavior run auto_apms_examples::demo::HelloWorld --blackboard name:=Turtle
    ```

Finally, you may as well run a cool **visual demonstration** of what's possible with this framework.

1. Install dependencies and build package `auto_apms_simulation`

    ```bash
    # Python packages for simulation (not all are available with rosdep)
    python3 -m pip install -r src/auto-apms/auto_apms_simulation/requirements.txt
    colcon build --packages-up-to auto_apms_simulation --symlink-install
    ```

1. Run the less intelligent behavior first

    ```bash
    source install/setup.bash
    ros2 launch auto_apms_simulation pyrobosim_hogwarts_launch.py
    # Press Ctrl+C to quit
    ```

    The actions of each robot you've seen are executed using behavior trees. This functionality is provided by the `auto_apms_behavior_tree` package. However, each robot is acting independently and they are not aware of their environment. Yet.

1. Now, we want to make the robots more intelligent and allow them to dynamically adjust their behavior when they encounter other robots inside one of the hallways. This is realized by implementing fallback mechanisms introduced by the `auto_apms_mission` package. To achieve that, add a launch argument

    ```bash
    source install/setup.bash
    ros2 launch auto_apms_simulation pyrobosim_hogwarts_launch.py mission:=true
    # Press Ctrl+C to quit
    ```

    The robots dynamically decide to retreat and wait until the hallway they are about to cross is not occupied anymore. They basically monitor if a certain event occurs and initialize a corresponding sequence of action if applicable. With this, we effectively introduced automatically orchestrated reactive behaviors.

    https://github.com/user-attachments/assets/adbb7cab-1a9b-424b-af61-61c351986287

# 🎓 Documentation

Make sure to visit the [User Guide](https://robin-mueller.github.io/auto-apms-guide/introduction/about) for tutorials and best practices when writing software using AutoAPMS.

We also offer an extensive [API Documentation](https://robin-mueller.github.io/auto-apms/) which is created using Doxygen >= 1.10. To generate the documentation run the following from the repository's root:

```bash
doxygen doc/Doxyfile
```

# 🙏 Acknowledgements

This repository builds on:

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) by Davide Faconti
- [pyrobosim](https://github.com/sea-bass/pyrobosim) by Sebastian Castro

Special thanks to the maintainers of these projects. It was their contributions to the robotic community that made AutoAPMS possible in the first place! 🚀
