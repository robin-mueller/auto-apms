![image](https://github.com/robin-mueller/auto-apms-guide/blob/master/public/logo/logo-wo-bg.png)

[![Website](https://img.shields.io/website?url=https://robin-mueller.github.io/auto-apms-guide/)](https://robin-mueller.github.io/auto-apms-guide/)
[![License](https://img.shields.io/github/license/robin-mueller/auto-apms?color=blue)](https://www.apache.org/licenses/LICENSE-2.0)

[![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)](https://github.com/robin-mueller/auto-apms/releases)
[![Build and Test](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yml)

# AutoAPMS - Smarter Missions with Safer Outcomes

**Auto**mated **A**ction **P**lanning and **M**ission **S**afeguarding for robotic applications.

With this ROS 2 package, the goal is to offer a comprehensible framework for implementing reactive behaviors during autonomous missions while focusing on PX4 applications. Additionally, this package facilitates the model-based development process of flight actions by introducing effective tools and methods for programmers.

> [!NOTE]
> This repository is currently a work in progress.

AutoAPMS provides PX4 developers with
- A generic approach for implementing tasks within the ROS 2 middleware
- Various predefined tasks commonly performed by UASs
- A model-based flight behavior engine incorporating behavior trees
- CMake macros for registering behavior tree resources
- Useful command line tools for orchestrating the system

Vist the [User Guide](https://robin-mueller.github.io/auto-apms-guide/welcome/) for getting started.

# TODO

- ~Redesign CMake resource management to integrate better with ament_index~
- Update behavior executor class and integrate with cmake redesign
- Parse plugin config in [register_behavior_tree_file.cmake](./src/auto-apms/cmake/register_behavior_tree_file.cmake) to make model generation also depend on the libraries installed by upstream packages
- Polish contingency_management example to enable the development and execution of reactive behaviors for UAS missions
- Create comprehensive documentation of member functions and intended use cases
- Implement more detailed examples
- Implement unit tests
