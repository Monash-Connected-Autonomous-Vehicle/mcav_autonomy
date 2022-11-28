# Data Recording

Launch files and nodes to help with recording data (mainly using the ros2 bag command).

## Launch files

### `update_lidar_timestamp.launch.xml`

Since the ros2 bag command replays data with old timestamps, 
(See https://github.com/ros2/rosbag2/issues/1056)
this node listens to the provided topic and republishes data 
with updated timestamps.

Run as follows: 
- `ros2 launch data_recording update_lidar_timestamp.launch.xml bag:=/home/mcav/mcav_ws/rosbags/rosbag2_2022_10_18-06_07_38_0/rosbag2_2022_10_18-06_07_38_0.db3` 

or 

- `ros2 launch data_recording update_lidar_timestamp.launch.xml bag:=./rosbag2_2022_10_18-06_07_38_0.db3`

The path to the rosbag can be absolute or relative to the current directory of the terminal.