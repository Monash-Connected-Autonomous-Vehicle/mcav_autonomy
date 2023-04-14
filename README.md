# 💻 mcav_autonomy
The urban-driving autonomous stack of the Monash Connected Autonomous Vehicles team. Designed to run on a StreetDrone Twizy and in the CARLA simulation environment.

# Installation - Using Docker (recommended)

## Requirements
- Docker (install on Ubuntu using `sudo apt install docker.io`)
- [Nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit) (For optional GPU support) 

## Installation

- Clone the repository: 
```
git clone git@github.com:Monash-Connected-Autonomous-Vehicle/mcav_autonomy.git
```
- Run the container:
```
cd mcav_autonomy
docker/run.sh
```
- See [docker/README.md](docker/README.md) for more details on using docker in this project.

# Installation - From Source
## Requirements
- [Ubuntu 22.04](https://ubuntu.com/download/desktop)
- [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## Installation
- Create a workspace: `mkdir -p ~/mcav_ws/src && cd ~/mcav_ws/src`
- Install vcstool: `sudo apt install python3-vcstool` or `pip3 install vcstool`
- Clone this repo: `git clone git@github.com:Monash-Connected-Autonomous-Vehicle/mcav_autonomy.git`
- Change directory: `cd mcav_autonomy`
- Clone source dependencies: `vcs import external/ < ros.repos`
- Change directory: `cd ~/mcav_ws`
- Install ROS dependencies: `rosdep install --from-paths src -i -r -y`
- Build the packages: `colcon build --symlink-install`

# Project Structure

This should result in a directory structure similar to the following:
```
mcav_ws/                                                     
├── build
├── install
└── src
    └── mcav_autonomy
        ├── autonomy_launch         # Launch files for entire stack
        ├── data
        │   ├── pointclouds         # Pointcloud map files (.pcd)
        │   ├── rosbags             # Recorded rosbags
        │   └── waypoints           # Global waypoints (.csv) 
        ├── data_recording          # Launch files for creating rosbags
        ├── docker                  # Dockerfiles and run scripts
        ├── external                # External dependencies that come in source code form
        │   ├── SD-VehicleInterface # Sends actuation commands, reads GPS/IMU and speedometer
        │   └── kiss-icp            # Performs lidar-based odometry (see github.com/PRBonn/kiss-icp)
        ├── mcav_interfaces         # Shared ROS Message and Service definitions
        ├── project_tracker         # Object detection and tracking
        ├── pure_pursuit            # Control system
        ├── sensors_launch          # Launch files for the sensors
        ├── simulation              # CARLA & simple kinematics simulation setups
        ├── velocity_planner        # Local planner
        └── visualisation
            └── vehicle_model       # 3D vehicle models for visualisation 
```

Pointcloud maps, rosbags, waypoints and other files that are data, not code, should be stored in `data/pointclouds`, `data/rosbags`, `data/waypoints` etc.

## External Dependencies

This project relies on:
- [MCAV's fork of the StreetDrone Vehicle Interface](https://github.com/Monash-Connected-Autonomous-Vehicle/SD-VehicleInterface) for communication with the vehicle actuation
- [KISS-ICP](https://github.com/PRBonn/kiss-icp) for lidar-based odometry

# Running

## How to run in CARLA
Terminal 1 (Launch CARLA server):
- `/opt/carla-simulator/CarlaUE4.sh`

Terminal 2 (Launch CARLA ros-bridge):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`

Terminal 3 (Launch autonomy stack):
- `cd ~/mcav_ws && source install/setup.bash`
- `ros2 launch autonomy_launch carla.launch.py waypoints_file:=/home/mcav/Sheng/control_ws/town01_path1.csv`

## How to run on Alienware

Setup CAN and enter Docker container:
```
./main_launch.sh
```

Launch the stack:
```
src
ros2 launch autonomy_launch streetdrone.launch.xml
```
