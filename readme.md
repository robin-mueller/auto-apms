# PX4 Behavior-based Control Library

A human-friendly mission control framework that allows to create arbitrary flight behaviors.**

*Currently work in progress*

With this ROS 2 package, the goal is to offer a control interface similar to [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html) but with greater focus on PX4 applications and more flexibility and user-friendliness. The package is also supposed to facilitate the model-based development process of flight tasks by introducing effective tools and methods for creating highly modular systems.

This repository provides PX4 developers with
- Generic interface for implementing tasks within the ROS 2 middleware
- Various predefined tasks commonly performed by UASs
- Model-based flight behavior engine incororating behavior trees
- CMake macros for registering behavior tree resources
- Useful command line tools for orchestrating the system

## ToDo

- Redesign CMake resource management to integrate better with ament_index
- Implement more detailed examples and automate their execution
- Implement unit tests
- Create comprehensive documentation of member functions and intended use cases

## Known Issues

- On the development system, the first message of `/fmu/out/mode_completed` was usually not received. In that case, the first task to be performed after starting the simulation will abort when completed. After retrying once, everything works fine.