# StreetDrone URDF Model

This model was adapted from https://github.com/streetdrone-home/SD-TwizyModel 

Changes:
- The wheel joints had to be changed from continuous/revolute to fixed.

- The file references were made relative to the package share directory.

- The colour was manually added.

- A launch file was created to load the model onto the `/robot_description` topic.

The model is made up of .xacro files that are interpreted by the xacro library, which expands them into
.urdf files. Xacro files can define macros and variables but are otherwise pretty similar to urdf. The velodyne and main chassis are .dae / .stl files.

# How to run

`colcon build --packages-select mcav_sd_model`

`source ~/colcon_ws/install/setup.bash`

`ros2 launch mcav_sd_model mcav_sd_model.launch.py`

or, include it in a launch file:

### `my_launch_file.launch.xml`

`<include file="$(find-pkg-share mcav_sd_model)/launch/mcav_sd_model.launch.py">`

To view result, open rviz, Add, By Display Type, RobotModel and change Description Topic to `/robot_description`