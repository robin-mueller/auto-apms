<p align="center" width="100%">
    <img width="70%" src="https://robin-mueller.github.io/auto-apms-guide/logo/logo.png">
</p>

<div align="center">

<h3>Automated Action Planning and Mission Safeguarding for Robotics</h3>

<a href="https://robin-mueller.github.io/auto-apms-guide/">![Website](https://img.shields.io/website?url=https%3A%2F%2Frobin-mueller.github.io%2Fauto-apms-guide&label=Website)</a>
<a href="https://doi.org/10.5220/0012951500003822">![DOI](https://zenodo.org/badge/DOI/10.5220/0012951500003822.svg)</a>
<a href="https://github.com/robin-mueller/auto-apms/releases">![Release](https://img.shields.io/github/v/release/robin-mueller/auto-apms?label=Release)</a>
<a href="https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml">![humble](https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml/badge.svg)</a>

</div>

# 💡 Motivation and Features

AutoAPMS is a ROS 2 software development framework offering an end-to-end solution for enabling autonomous robotic operation. It can be applied in any field of robotics as long as the corresponding systems are running ROS 2. Other popular middlewares like [PX4](https://px4.io/) are also supported (they must offer a possibility for bridging internal messages to ROS 2 topics).

Developers of real-time systems benefit from:

- Tools and methods for designing cooperative systems
- Modular, plugin-based approach for implementing robotic skills/tasks
- User-friendly task planning system adopting the behavior tree paradigm
- Standardized, highly configurable behavior tree executor
- Powerful behavior tree builder API
- Automated contingency and emergency management system
- Useful command line tools for running and orchestrating missions

All of these features reduce the amount of boilerplate code required when creating functional robotic applications and make the complex software development process significantly easier, faster and less error prone.

For more information and an extensive how-to guide, feel encouraged to visit the 👉 [**AutoAPMS Website**](https://robin-mueller.github.io/auto-apms-guide/) 👈.

# 🚀 Setup and Demonstration

AutoAPMS is **designed for Linux**. The following ROS 2 versions are supported:

| ROS 2 Version | OS | Status |
| :-------------: | :-----------: | :-----------: |
| [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) | [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/) | [![ROS 2 Humble Test](https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml/badge.svg)](https://github.com/robin-mueller/auto-apms/actions/workflows/humble.yaml) |

The following installation guide helps you getting started with AutoAPMS by building the source code yourself. Finally, you may test your installation by running an example.

Firstly, you have to create a [ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository.

```bash
mkdir ros2_ws && cd ros2_ws
(mkdir src && cd src && git clone https://github.com/robin-mueller/auto-apms.git)
```

Afterwards, install all required dependencies. We assume that you already installed ROS 2 on your system.

```bash
rosdep init  # Skip this if rosdep has already been initialized
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Then, build and install all of the source packages up to `auto_apms_examples`.

> [!NOTE]
> We highly recommend building your workspace using the `symlink-install` option since AutoAPMS extensively utilizes XML and YAML resources. This option installs symbolic links to those non-compiled source files meaning that you don't need to rebuild again and again when you're for example tweaking a behavior tree document file. Instead, your changes take effect immediately and you just need to restart your application.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to auto_apms_examples --symlink-install
```

Congratulations, you've already successfully installed all necessary resources. You may now launch a lightweight simulation that applies the concepts offered by AutoAPMS. This should give you an idea of what's possible with this framework.

The basic robot behavior can be observed by executing

```bash
source install/setup.bash
ros2 launch auto_apms_examples pyrobosim_hogwarts_launch.py
# Press Ctrl+C to quit
```

The actions of each robot you've seen are executed using behavior trees. This functionality is provided by the `auto_apms_behavior_tree` package. However, each robot is acting independently and they are not aware of their environment. Yet.

Now, we want to make the robots more intelligent and allow them to dynamically adjust their behavior when they encounter other robots inside one of the hallways. This is realized by implementing fallback mechanisms introduced by the `auto_apms_mission` package. To achieve that, you simply have to specify the following launch argument.

```bash
source install/setup.bash
ros2 launch auto_apms_examples pyrobosim_hogwarts_launch.py mission:=true
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

# 🌟 Credits

Aside from the core ROS 2 packages, this repository builds upon

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) by Davide Faconti.

- [pyrobosim](https://github.com/sea-bass/pyrobosim) by Sebastian Castro.

Thanks for all the great work from the maintainers and contributors of the above mentioned projects.
