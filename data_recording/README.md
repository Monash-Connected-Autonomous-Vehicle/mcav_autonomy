# Data Recording and Playback

Launch files and nodes to help with recording and playing back data (mainly using the ros2 bag command).

## Launch files

### `record.launch.xml`

Runs `ros2 bag record` to record IMU, GPS, current twist, CAN frames (transmitted and received), and lidar points (lidar can be enabled by adding lidar:=true to the launch command)

To record without lidar points: `ros2 launch data_recording record.launch.xml`

To record with lidar points: `ros2 launch data_recording record.launch.xml lidar:=True`

To launch the sd_vehicle_interface, append `launch_vi:=True` to the command.

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