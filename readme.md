# PX4 Behavior-based Control Library

This repo is currently a work in progress.

With this ROS 2 package, the goal is to provide a control interface similar to [Nav2 Behavior Trees](https://docs.nav2.org/behavior_trees/index.html) but with greater focus on PX4 applications and more flexibility and user-friendliness. The package is also supposed to facilitate the model-based development process of flight tasks by introducing effective tools and methods for creating highly modular systems.

## Known Issues

- On the development system, the first message of `/fmu/out/mode_completed` was usually not received. In that case, the first task to be performed after starting the simulation will abort when completed. After retrying once, everything works fine.