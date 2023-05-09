# 🦺 Supervisor

The supervisor monitors the health of the software and hardware stack on the StreetDrone.

It is designed to:

- check that all necessary nodes remain alive
- monitor CPU/GPU/RAM/disk space (not yet implemented)
- monitor for any error states from our software (not yet implemented)
- monitor the connection to the StreetDrone (not yet implemented)
- check that sensors are correctly returning data (not yet implemented)

As long as no problems are found, it sends the required heartbeat to the StreetDrone and publishes `True` on the `/supervisor/is_okay` topic.

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
