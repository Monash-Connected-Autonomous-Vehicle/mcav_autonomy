# 🦺 Supervisor

The supervisor monitors the health of the software and hardware stack on the StreetDrone.

It is designed to:

- check that all necessary nodes remain alive
- monitor CPU/GPU/RAM/disk space (not yet implemented)
- monitor for any error states from our software (not yet implemented)
- monitor the connection to the StreetDrone (not yet implemented)
- check that sensors are correctly returning data (not yet implemented)

As long as no problems are found, it sends the required heartbeat to the StreetDrone and publishes `True` on the `/supervisor/is_okay` topic.

## Requirements

- [Ubuntu 20.04](https://ubuntu.com/download/desktop)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- A colcon workspace (assumed to be at `~/colcon_ws`). See the [Creating a Workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) tutorial

## Installation

- Go to the `src` directory: `cd ~/colcon_ws/src`
- Clone the source code: `git clone `
- Go to the root of the workspace: `cd ~/colcon_ws`
- Build: `colcon build --packages-select --symlink-install supervisor`

## Usage

The names of required nodes must be added to the `required_nodes` list in the parameters of `launch/supervisor.launch.py` as shown:

```python
Node(
	package='supervisor',
	executable='supervisor',
	name='supervisor',
	parameters=[{
		'required_nodes': ['minimal_publisher',]
	}]
),
```

Run the supervisor using `ros2 launch supervisor supervisor.launch.py`
