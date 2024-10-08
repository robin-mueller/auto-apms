<p align="center" width="100%">
    <img width="70%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>

[![Website](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=Website)](https://robin-mueller.github.io/auto-apms-guide/)
[![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)](https://github.com/robin-mueller/auto-apms/releases)
[![Build and Test](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml)

**Auto**mated **A**ction **P**lanning and **M**ission **S**afeguarding for Robotics.

> [!NOTE]
> This repository is currently a work in progress.

# AutoAPMS Mission Management Software

With this ROS 2 package, the goal is to offer a comprehensible framework for designing and executing autonomous robotic missions. The user is provided with effective tools and methods for software development. The repository focuses on [PX4](https://px4.io/) applications, since enabling safe autonomous missions executed by unmanned aerial systems is a hot research topic at the moment. However, the framework can be applied in any field of robotics as long as the corresponding systems are running ROS 2.

In the first place, AutoAPMS provides PX4 developers with

- A generic approach for implementing tasks within the ROS 2 middleware
- Various predefined tasks commonly performed by UASs
- An action-based flight behavior engine incorporating behavior trees
- An efficient Behavior Tree resource system integrating with `ament_index`
- Useful command line tools for orchestrating the system

For more information, please visit the [official documentation](https://robin-mueller.github.io/auto-apms-guide/intro).

## TODO

- Update behavior executor class and integrate with cmake redesign
- Polish contingency_management example to enable the development and execution of reactive behaviors for UAS missions
- Create comprehensive documentation of member functions and intended use cases
- Implement more detailed examples
- Implement unit tests
