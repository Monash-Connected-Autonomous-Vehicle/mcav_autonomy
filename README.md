# 💻 mcav_autonomy
The urban-driving autonomous stack of the Monash Connected Autonomous Vehicles team. Designed to run on a StreetDrone Twizy and in the CARLA simulation environment.

# Requirements
- [Ubuntu 22.04](https://ubuntu.com/download/desktop)
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

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
        ├── data_recording          # Launch files for creating rosbags
        ├── docker                  # Dockerfiles and run scripts
        ├── mcav_interfaces         # Shared ROS Message and Service definitions
        ├── mcav_sd_model           # 3D vehicle models for visualisation
        ├── mcav_sd_sensing         # Launch files for the sensors
        ├── project_tracker         # Object detection and tracking
        ├── pure_pursuit            # Control system
        ├── simulation              # CARLA simulation setup
        └── velocity_planner        # Local planner
```
See [Git Tools - Submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) for information on working with submodules.

# How to run in CARLA
Terminal 1 (Launch CARLA server):
- `/opt/carla-simulator/CarlaUE4.sh`

Terminal 2 (Launch CARLA ros-bridge):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`

Terminal 3 (Launch autonomy stack):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch autonomy_launch carla.launch.py waypoint_filename:=/home/mcav/Sheng/control_ws/town01_path1.csv`

# How to run on PX2
Check out the tutorial [here](https://www.notion.so/monashcav/ROS-ROS2-Bridge-Docker-on-the-PX2-b467b22b85444f27a47ded13b8968370). 

Terminal 1 outside the container (Launch camera and lidar drivers):
- `autonomy_launch/start_cam_lidar.sh`

Terminal 2 (Enter the container): 
- `docker/px2_run.sh`

Terminal 2a (Start the ros1 bridge):
- `autonomy_launch/ros1bridge.sh`

Terminal 2b (Launch the mcav_autonomy software):
- `ros2 launch autonomy_launch autonomy.launch`
