# 💻 mcav_autonomy
The urban-driving autonomous stack of the Monash Connected Autonomous Vehicles team. Designed to run on a StreetDrone Twizy and in the CARLA simulation environment.

# Requirements
- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)

## Simulation Requirements

## StreetDrone Requirements

# Setup
- Create a workspace: `mkdir -p ~/mcav_ws/src && cd ~/mcav_ws/src`
- Clone this package: `git clone git@github.com:Monash-Connected-Autonomous-Vehicle/mcav_autonomy.git --recurse-submodules` (if you forget to `--recurse-submodules`, you can later run `git submodule update --init --recursive` instead)
- Install ROS dependencies: `cd .. && rosdep install --from-paths src --ignore-src -r -y`
- Build the packages: `colcon build --symlink-install`

This should result in a directory structure similar to the following:
```
mcav_ws/                                                     
├── build                                                                                                               
└── src
    └── mcav_autonomy
        ├── autonomy_launch         # Launch files for entire stack
        ├── mcav_interfaces         # Shared ROS Message and Service definitions
        ├── project_tracker         # Object detection and tracking
        ├── pure_pursuit            # Control system
        ├── simulation              # CARLA simulation setup
        └── velocity_planner        # Local planner
```
See [Git Tools - Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) for information on working with submodules.

# How to run
Terminal 1 (Launch CARLA server):
- `/opt/carla-simulator/CarlaUE4.sh`

Terminal 2 (Launch CARLA ros-bridge):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`

Terminal 3 (Launch autonomy stack):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch autonomy_launch carla.launch.py waypoint_filename:=/home/mcav/Sheng/control_ws/town01_path1.csv`
