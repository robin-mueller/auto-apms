![image](https://github.com/robin-mueller/px4-behavior-docs/blob/master/public/logo-wo-bg.png)

[![Website](https://img.shields.io/website?url=https://robin-mueller.github.io/px4-behavior-docs/)](https://robin-mueller.github.io/px4-behavior-docs/)
[![License](https://img.shields.io/github/license/robin-mueller/px4-behavior?color=blue)](https://www.apache.org/licenses/LICENSE-2.0)

[![Release](https://img.shields.io/github/v/release/robin-mueller/px4-behavior)](https://github.com/robin-mueller/px4-behavior/releases)

# PX4 Behavior | User-friendly Robotic Task Management

*Currently work in progress*

With this ROS 2 package, the goal is to offer a control interface similar to [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html) but with greater focus on PX4 applications and more flexibility and user-friendliness. The package is also supposed to facilitate the model-based development process of flight tasks by introducing effective tools and methods for creating highly modular systems.

This repository provides PX4 developers with
- A generic approach for implementing tasks within the ROS 2 middleware
- Various predefined tasks commonly performed by UASs
- A model-based flight behavior engine incorporating behavior trees
- CMake macros for registering behavior tree resources
- Useful command line tools for orchestrating the system

Vist the [User Guide](https://robin-mueller.github.io/px4-behavior-docs/introduction/) for getting started.

# ToDo

- ~Redesign CMake resource management to integrate better with ament_index~
- Update behavior executor class and integrate with cmake redesign
- Make contingency_management example more abstract to represent a general operations engine for UAS (ops_engine)
- Create comprehensive documentation of member functions and intended use cases
- Implement more detailed examples and automate their execution
- Implement unit tests


# Known Issues

- On the development system, the first message of `/fmu/out/mode_completed` was usually not received. In that case, the first task to be performed after starting the simulation will abort when completed. After retrying once, everything works fine.
