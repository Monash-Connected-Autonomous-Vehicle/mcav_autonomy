# Simulation

This package contains nodes and launch files related to simulation.

## `simple_sim.py`

This simulator is for evaluating planning and control in isolation. (i.e. without sensing, perception, localisation).

To run:
```
. ~/mcav_ws/install/setup.bash
ros2 launch simulation simple_sim.launch.xml
ros2 launch simulation simple_sim.launch.xml waypoint_filename:=/home/mcav/mcav_ws/src/mcav_autonomy/simulation/example_waypoints/straight.csv
```