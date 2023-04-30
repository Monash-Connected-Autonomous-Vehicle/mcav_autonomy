# Sensors

This package contains launch files for the sensors of the car.

## Running LiDAR
Make sure the workspace has been built and sourced. 
`ros2 launch sensors_launch velodyne.launch.xml launch_velodyne_driver:=true`

## Running IMU Calibration
Make sure the workspace has been built and sourced.
`ros2 launch sensors_launch imu_calibration.launch.xml`
This spins up the imu_calibration node and the socketcan_sender and socketcan_reciever nodes. Then sends the calibration messages as Can Frames to the sensor.

### The imu_calibration node
This node is a publisher that sends a Can Frame to the /to_can_bus
topic, it sends the message ever 0.5s continously until the script
is closed. The Frame is message sent to the write imu sensor (657h)
setting it's X, Y and Z acceleration. The main function spins the node with it's default params - X = 0G, Y = 0G and Z = 1G.

To change these values, set the parameters (X, Y or Z) in ***CalibrationPublisher()*** as follows:
- 0 = 0G
- 1 = +1G
- 2 = -1G
