<p align="center" width="100%">
    <img width="70%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>

[![Website](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=Website)](https://robin-mueller.github.io/auto-apms-guide/)
[![DOI](https://zenodo.org/badge/DOI/10.5220/0012951500003822.svg)](https://doi.org/10.5220/0012951500003822)
[![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)](https://github.com/robin-mueller/auto-apms/releases)
[![Build and Test](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/build-and-test.yaml)

**Auto**mated **A**ction **P**lanning and **M**ission **S**afeguarding for Robotics.

# Features and Motivation

AutoAPMS is a ROS 2 software development framework offering convenient methods and tools to create autonomous robotic missions. It can be applied in any field of robotics as long as the corresponding systems are running ROS 2. Additionally, other popular middlewares like [PX4](https://px4.io/) are supported as well if they offer a possibility for bridging internal messages to ROS 2 topics.

Key features of the packages in this repository are

- Generic ROS 2 node design for implementing real-time tasks
- Modular, plugin-based approach for implementing robotic skills/actions
- Standardized, highly configurable behavior tree executor
- Powerful behavior tree builder C++ API
- Automated contingency and emergency management system
- Useful command line tools for running and orchestrating missions

All of these features reduce the amount of boilerplate code required for implementing functional robotic missions and make the complex software development process significantly easier, faster and less error prone. For more information and an extensive how-to guide, feel encouraged to visit the [user guide](https://robin-mueller.github.io/auto-apms-guide/intro).

# API Reference

The source code documentation is created using Doxygen >= 1.10 and deployed using GitHub pages. To generate the documentation run the following from the repository's root:

```sh
doxygen doc/Doxyfile
```
