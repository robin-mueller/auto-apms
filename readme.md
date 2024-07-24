# PX4 Behavior-based control library



## Known Issues

- On the development system, the first message of `/fmu/out/mode_completed` was usually not received. Therefore, the first maneuver/action from the `commander` package to be performed after starting the simulation will abort when completed. After retrying once, everything works fine.