# 📜 mcav_interfaces

This package provides custom message and service definitions that are used in multiple packages across the mcav_autonomy software stack.

<p align="center">
  <img width="501" alt="An example visualisation of the Waypoint and DetectedObject messages" src="https://user-images.githubusercontent.com/7232997/235609518-885f3950-9dea-46e7-a768-ba16abd3a999.png">
  <br />
  <em>Figure 1: An example visualisation of the Waypoint (green circles) and DetectedObject (grey box) messages.</em>

</p>

## Messages

### DetectedObject
Represents an object in 3D space. The object is defined by its bounding box and a class label. The bounding box is defined by its center point and its dimensions. The class label is defined by an integer value. The class labels are defined in the `DetectedObject.msg` file as constants.

Each object possess a unique ID, which is assigned by the object tracker. The ID is used to track the object over time.

See [DetectedObject.msg](./msg/DetectedObject.msg) for details.

### DetectedObjectArray
Represents a list of detected objects.

### Waypoint
Represents a waypoint in 3D space. The waypoint is defined by its position and a desired velocity at that point.

See [Waypoint.msg](./msg/Waypoint.msg) for details.

### WaypointArray
Represents a list of waypoints that form a trajectory. That is: a path to follow, and desired velocities at each point along that path. 

## Usage

- Add as a dependency by adding `<exec_depend>mcav_interfaces</exec_depend>` (python) or `<build_depend>mcav_interfaces</build_depend>` (cpp) to your `package.xml`.
```python
from mcav_interfaces.msg import DetectedObject
...
self.publisher_ = self.create_publisher(DetectedObject, 'topic', 10)
...
msg = DetectedObject()
msg.object_class = DetectedObject.CLASS_BICYCLE
self.publisher_.publish(msg)
```

## Visualisation
The `waypoint_visualiser` and `object_visualiser` nodes can be used to visualise the `DetectedObjectArray` and `WaypointArray` messages respectively. The visualisation can be viewed in RViz. See Figure 1 for an example.
