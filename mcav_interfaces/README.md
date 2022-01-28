# 📜 mcav_interfaces

This package provides custom message and service definitions to be used in the software stack.

## Installation
- Go to the `src` directory: `cd ~/colcon_ws/src`
- Clone the source code: `git clone <git-url>`
- Go to the root of the workspace: `cd ~/colcon_ws`
- Build: `colcon build --packages-select --symlink-install mcav_interfaces`
- Run `ros2 interface show mcav_interfaces/msg/DetectedObject` to check successful build and see message definition.

## Usage

- Add as a dependency by adding `<exec_depend>mcav_interfaces</exec_depend>` to your `package.xml`.
```python
from mcav_interfaces.msg import DetectedObject
...
self.publisher_ = self.create_publisher(DetectedObject, 'topic', 10)
...
msg = DetectedObject()
msg.object_class = DetectedObject.CLASS_BICYCLE
self.publisher_.publish(msg)
```