# Localisation using the lidar to correct an initial pose estimate using ICP on lidar readings compared to a reference map

## Requirements
- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- [Open3D](http://www.open3d.org/docs/release/introduction.html).
  
## Installation

- Go to the `src` directory: `cd ~/colcon_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/colcon_ws`
- Build: 'colcon build --packages-select localisation

## Usage
### Inputs
- reference pointcloud map ('geometry_msgs/PointCloud2' on topic '/pointcloud_map')
- lidar pointcloud ('geometry_msgs/PointCloud2' on topic '/lidar_points')
- initial pose guess from the reference map ('geometry_msgs/PoseStamped' on topic '/initial_pose_guess')
### Outputs
- A a transform from the reference pointcloud frame to the frame of the lidar ('geometry_msgs/TransformStamped' on topic '/initial_pose_transform')

### Run the initialise_pose node when starting autonomous motion
- `cd ~/colcon_ws && . install/setup.bash`
- `ros2 run localisation initialise_pose`
