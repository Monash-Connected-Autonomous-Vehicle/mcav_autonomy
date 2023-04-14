# Simulation

This package contains nodes and launch files related to simulation.

## `simple_sim.py`

This simulator is for evaluating planning and control in isolation. (i.e. without sensing, perception, localisation).

To run:
```
. ~/mcav_ws/install/setup.bash
ros2 launch simulation simple_sim.launch.xml waypoints_file:=/home/mcav/mcav_ws/src/mcav_autonomy/simulation/example_waypoints/straight.csv
```

## `twist_stamp_remover.py`

This is a small node that accepts a [TwistStamped](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/TwistStamped.msg) message, and republishes the same values in a [Twist](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg) message, without the stamp. This allows for interoperability of our stack that uses TwistStamped messages (SD-VehicleInterface and pure_pursuit), with CARLA and gazebo, which use Twist messages.

Input and output topics can be specified using "remap" tags when including the node in a launch file. For example usage, see `twist_stamp_remover.launch.xml`.