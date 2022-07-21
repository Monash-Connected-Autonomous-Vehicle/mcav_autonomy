#!/bin/bash

# READ THE FOLLOWING!!
# This is script runs the camera and LiDAR divers. This should be ran on the native Ubuntu 16.04 system on the PX2 and NOT IN THE CONTAINER ENVIRONMENT.

function pause(){
	read -t 7 -p 'waiting for drivers to launch completely!'
	echo ""
}

# RUN CAMERAS
# -----------
# - compressed images
# - downsampled
# - throttled to 10Hz
gnome-terminal -- ~/mcav_autonomy/autonomy_launch/camera_drivers_px2.sh

# pause until drivers completely launch 
# to avoid ROS master clash.
pause 

# RUN LiDAR
gnome-terminal -- roslaunch velodyne_pointcloud VLP16_points.launch

