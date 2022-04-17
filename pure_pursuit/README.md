# What is pure pursuit?
Project Pure Pursuit will be our base for our control algorithm for the street drone. it would be accompanied by a state machine that uses Object Tracking and Waypoints to handle edge cases.

Pure Pursuit is a tracking algorithm which works by calculating the amount of curvature required to move a vehicle from its current position to some goal position. It tracks a reference path (made of waypoints) using the geometry of the vehicle kinematics as well as a fixed distance known as the look ahead distance. Ultimately, the vehicle chases a look ahead point which constantly changes as the vehicle moves. This look ahead point changes based on the current position of the vehicle until the last waypoint on the path and has a minimum distance of the look ahead relative to the vehicle. 

# General notes regarding this package
- This package was designed to be compatible with MCAV's velocity_planner package which is capable of generating local waypoints. 

# Requirements:
- Ubuntu 20.0.4
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [CARLA 0.9.13](https://carla.readthedocs.io/en/latest/start_quickstart/#carla-installation) 
- [CARLA ROS Bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/)
- [MCAV interfaces](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav_interfaces) should be in the same workspace
- (Optional) [Velocity planner](https://github.com/Monash-Connected-Autonomous-Vehicle/velocity_planner) should be in the same workspace

# How to launch simulation:
### 1. Launch CARLA (new terminal)
    cd <path-to-carla>
    ./CarlaUE4.sh
### 2. Launch ROS bridge (new terminal)
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
    cd <path-to-ros-bridge-workspace>
    source install/setup.bash
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
### 3. Launch twist to control package (new terminal)
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
    cd <path-to-ros-bridge-workspace>
    source install/setup.bash
    ros2 launch carla_twist_to_control carla_twist_to_control.launch.py
### 4. Install pure pursuit package and build workspace
    cd <path-to-workspace>
    git clone https://github.com/Monash-Connected-Autonomous-Vehicle/mcav_interfaces.git
    colcon build
### 5. Spawn vehicle and generate waypoints on predefined course (new terminal)
#### Important: 
velocity_planner's waypoint_reader node is able to publish global waypoints from a .csv file. In order for that script to work properly with pure_pursuit, a section of carla_global_planner's callback function will have to be commented out. The secton is annotated by a "TODO" note in the commenting. Otherwise, carla_global_planner.py should be able to generate its own waypoints in CARLA and publish them as a global_wapoints topic to be fed to velocity_planner.py.

    cd <path-to-workspace>
    export CARLA_ROOT=<path-to-carla>
    export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-<carla_version_and_arch>.egg:$CARLA_ROOT/PythonAPI/carla
    source install/setup.bash
    ros2 launch pure_pursuit purepursuit_simulation.launch.py
    
# Contact
Sheng (Senman) Qiu - sqiu0004@student.monash.edu; senmanqiu@gmail.com

