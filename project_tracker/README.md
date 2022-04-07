# Project Tracker

Project tracker takes inputs from the @Multi-Task Panoptic Perception model and LiDAR sensor and fuses them together to create accurate pose and velocity estimates of the object over time in 3D space.

![tracker](https://user-images.githubusercontent.com/69286161/151936865-c160a7b6-f4cc-4b03-b0d2-b586d1aff493.gif)



## Getting Started

### prerequisites
* create a ros2 workspace by following [these](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) instructions
* clone this repository into the src directory in your ros2 workspace

### Installation

1. Download synced+rectified data from [here](http://www.cvlibs.net/datasets/kitti/raw_data.php) or to download the data directly, press [this](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0048/2011_09_26_drive_0048_sync.zip) download button
2. After the download is complete, navigate to ```/home/mcav/DATASETS/KITTI/```, unzip the downloaded zip and copy the folder named `2011_09_26` into the `KITTI` directory. Your tree for the binary files should then look like: `/home/mcav/DATASETS/KITTI/2011_09_26/2011_09_26_drive_0048_sync/velodyne_points/data/`
3. Install dependencies
    ```sh
    sudo apt install python3-pcl
    sudo apt-get install ros-galactic-sensor-msgs-py
    sudo pip3 install transforms3d
    ```
3. Navigate to the root folder of your workspace
    ```sh
    cd YOUR_WORKSPACE_ROOT
    ```
4. Build the package
	```sh
    colcon build
    . install/setup.bash
    ```
5. In a new terminal, navigate to the root of your workspace and call the publisher to publish mock data.
	```sh
    cd YOUR_WORKSPACE_ROOT
    . install/setup.bash
    ros2 run project_tracker mock_pub
    ```
6.  Refer to the [pcl_bind](https://github.com/Monash-Connected-Autonomous-Vehicle/pcl_bind) package to run the `filter` node to reduce the number of LiDAR points.
7.  In a new terminal, call the clustering node to produce clusters, bounding boxes and `DetectedObjectArray`
    ```sh
    . install/setup.bash
    ros2 run project_tracker cluster
    ```

#### Parameters and ROS Info

##### Parameters to modify for clustering: 
* `self.min_cluster_size`: minimum number of LiDAR points to be considered a cluster
* `self.max_cluster_size`: maximum number of LiDAR points to be considered a cluster
* `self.cluster_tolerance`: unsure exactly what the interpretation of this parameter is. I originally thought it was the maximum search radius from any point to another but think it is more nuanced than that after playing with it.

##### Topics
|Topic|Type|Objective|Nodes interacting|
------|----|---------|-----------------|
|`/velodyne_points`|`sensor_msgs.msg PCL2`|Publish mock LiDAR data|`project_tracker::mock_pub` publishes, `pcl_bind::filter` subscribes|
|`clustered_pointclouds`|`sensor_msgs.msg PCL2`|View clustered pointclouds in PCL2 format for visualisation|`project_tracker::cluster` publishes|
|`bounding_boxes`|`visualization_msgs.msg MarkerArray`|View bounding boxes over identified clusters for visualisation|`project_tracker::cluster` publishes|
|`detected_objects`|`mcav_interfaces::DetectedObjectArray`|Emit detected objects for use in other MCAV nodes e.g. path planning|`project_tracker::cluster` publishes|

## Carla Integration

1. Start Carla Agent `/opt/carla-simulator/CarlaUE4.sh`
2. Source carla_ros_bridge repo
```bash
cd <PATH-TO-carla_ros_bridge> (on the beauty this is ~/Sheng/carla_ros_bridge, the beast it is ~/liam_ws/carla_ros_bridge, I think)
source  ./install/setup.bash
```
3. Launch carla_ros_bridge `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`
4. Run python setup script `python3 <PATH-TO-project_tracker>/project_tracker/tracking_carla_setup.py`
5. Open rviz and set frame_id to `ego_vehicle`, add pointcloud from `/carla/ego_vehicle/front/lidar` and camera from `/carla/ego_vehicle/front/rgb_front`

## Contact
Amir Toosi - amir.ghalb@gmail.com

Ben Edwards - bedw0004@student.monash.edu
