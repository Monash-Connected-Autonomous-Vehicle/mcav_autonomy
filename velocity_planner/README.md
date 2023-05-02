# 🧶 Velocity Planner
The velocity planner is designed to take in fixed global waypoints from a map, and the current world state (vehicle location, detected obstacles, detected traffic signals) and alter the velocities of the waypoints as required.

![View in rviz of the planner output](screenshots/velocity_planner.png)
Yellow cubes: global waypoints, Purple circle: current pose, Green/red circles: local planned waypoints

## Requirements

- [Ubuntu 22.04](https://ubuntu.com/download/desktop)
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [mcav_interfaces](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav_interfaces)

## Installation

- Go to the `src` directory: `cd ~/mcav_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/mcav_ws`
- Build: `colcon build --symlink-install --packages-up-to velocity_planner`

## Usage
### Inputs
- current location (`geometry_msgs/Pose` on topic `/planner/current_pose`)
- global waypoints (`mcav_interfaces/msg/WaypointArray` on topic `/planner/global_waypoints`)

### Outputs
- Array of upcoming waypoints beginning at the point closest to the vehicle (`mcav_interfaces/msg/WaypointArray` on topic `/planner/local_waypoints`)
### Run the planner
- `cd ~/colcon_ws && . install/setup.bash`
- `ros2 launch velocity_planner velocity_planner.launch.py`

### Visualise global and local waypoints
- `cd velocity_planner/`
- `rviz2 -d planner.rviz`

## Publishing waypoints
### 1. Test using fake global waypoints
- `. install/setup.bash`
- `ros2 run velocity_planner fake_waypoint_publisher`

### or 2. Test using waypoints from a csv file
- `. install/setup.bash`
- `ros2 run velocity_planner waypoint_reader town01_small_waypoints.csv`

## Example run 
Shows planner stopping for a detected object

https://user-images.githubusercontent.com/7232997/153693042-92fc1265-6817-4bae-8867-90b4cb17fd1c.mov

