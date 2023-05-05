# 🧶 Velocity Planner
The velocity planner is designed to take in fixed global waypoints from a map, and the current world state (vehicle location, detected obstacles, detected traffic signals) and alter the velocities of the waypoints as required.

This package also contains other nodes that are useful for testing and debugging the planner.

https://user-images.githubusercontent.com/7232997/153693042-92fc1265-6817-4bae-8867-90b4cb17fd1c.mov

Figure 1: View in rviz of the planner output

Yellow cubes: global waypoints, Green/red circles: local planned waypoints

## Requirements

- [Ubuntu 22.04](https://ubuntu.com/download/desktop)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [mcav_interfaces](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav_interfaces)

## Installation

- Go to the `src` directory: `cd ~/mcav_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/mcav_ws`
- Build: `colcon build --symlink-install --packages-up-to velocity_planner`

## Nodes

| Node | Description | Inputs | Outputs |
| --- | --- | --- | --- |
| `velocity_planner` | Takes in global waypoints and the current vehicle location and outputs a set of local waypoints | `/current_pose` - current location of the vehicle (`geometry_msgs/Pose`) <br> `/global_waypoints` - waypoints for the vehicle to follow (`mcav_interfaces/WaypointArray`) | `/local_waypoints` (`mcav_interfaces/WaypointArray`) |
| `object_visualiser` | Publishes visualisation markers so objects can be viewed in RViz | `/detected_objects` - a list of detected objects (`mcav_interfaces/DetectedObjectArray`) | `/detected_objects_marker_array` (`visualization_msgs/MarkerArray`) |
| `fake_waypoint_publisher` | Publishes an array of waypoints beginning at 0,0,0. Used for testing during development. | None | `/global_waypoints` (`mcav_interfaces/WaypointArray`) |
| `fake_object_publisher` | Publishes an object that oscillates back and forth. Used for checking that the planner reacts objects. | `/detected_objects` - a list of detected objects (`mcav_interfaces/DetectedObjectArray`) | `/detected_objects_marker_array` (`visualization_msgs/MarkerArray`) |
| `pose_estimate_to_tf` | Broadcasts a tf2 transform between map and base_link when a 2D Pose Estimate is sent in RViz. Useful for debugging the planner. | `/initialpose` - the pose topic that RViz publishes to when you use 2D Pose Estimate (`geometry_msgs/PoseStamped`) | `/current_pose` (`geometry_msgs/Pose`) <br> `/tf` - A transform from `map` to `base_link` frames using tf2 |
| `waypoint_reader` | Reads waypoint coordinates and velocities stored in a .csv file in the format exported by [Tier IV's Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) | A csv file as an argument | `/global_waypoints` (`mcav_interfaces/WaypointArray`) |

## Usage
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

## Publishing a fake object
### Test with a fake detected object (see Figure 1)
- `ros2 run velocity_planner fake_object_publisher`