![image](https://github.com/robin-mueller/auto-apms-guide/blob/master/public/logo/logo-wo-bg.png?raw=true)

[![Website](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=Website)](https://robin-mueller.github.io/auto-apms-guide/)
[![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)](https://github.com/robin-mueller/auto-apms/releases)
[![Build and Test](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yml)

# AutoAPMS - Smarter Missions with Safer Outcomes

**Auto**mated **A**ction **P**lanning and **M**ission **S**afeguarding for robotic applications.

With this ROS 2 package, the goal is to offer a comprehensible framework for implementing reactive behaviors during autonomous missions while focusing on PX4 applications. Additionally, this package facilitates the model-based flight action development process by introducing effective tools and methods for programmers.

> [!NOTE]
> This repository is currently a work in progress.

AutoAPMS provides PX4 developers with
- A generic approach for implementing tasks within the ROS 2 middleware
- Various predefined tasks commonly performed by UASs
- A model-based flight behavior engine incorporating behavior trees
- CMake macros for registering behavior tree resources
- Useful command line tools for orchestrating the system

Vist the [User Guide](https://robin-mueller.github.io/auto-apms-guide/introduction/overview) for getting started.

# TODO

- Update behavior executor class and integrate with cmake redesign
- Polish contingency_management example to enable the development and execution of reactive behaviors for UAS missions
- Create comprehensive documentation of member functions and intended use cases
- Implement more detailed examples
- Implement unit tests
