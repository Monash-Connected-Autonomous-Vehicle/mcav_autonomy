# mcav_autonomy
The urban-driving autonomous stack of the Monash Connected Autonomous Vehicles team. Designed to run on a StreetDrone Twizy and in the CARLA simulation environment.

# Requirements
- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- [vcstool](https://github.com/dirk-thomas/vcstool): `sudo apt install python3-vcstool`

## Simulation Requirements

## StreetDrone Requirements

# Setup
- Create a workspace: `mkdir -p ~/mcav_ws/src && cd ~/mcav_ws/src`
- Clone this package: `git clone git@github.com:owenbrooks/mcav_autonomy.git`
- Clone other mcav packages: `vcs import < src/mcav_autonomy/autonomy.rosinstall`
- Install ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Build the packages: `colcon build --symlink-install`

This should result in a directory structure similar to the following:
```
mcav_ws
└── src
    ├── mcav_autonomy
    ├── mcav_interfaces
    ├── project_tracker
    ├── pure_pursuit
    └── velocity_planner
```
To update the dependency list when adding or updating packages in the future, use `cd ~/mcav_ws/src/ && vcs export > mcav_autonomy/autonomy.rosinstall --exact`.

# How to run
- `ros2 launch mcav_autonomy carla.launch` (not yet included)
