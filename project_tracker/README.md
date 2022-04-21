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
    sudo apt-get install ros-galactic-tf-transformations
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
6.  In a new terminal, navigate to the root of your workspace and call the `filter` node to reduce the number of LiDAR points.
	```sh
    cd YOUR_WORKSPACE_ROOT
    . install/setup.bash
    ros2 run project_tracker filter
    ```
7.  In a new terminal, call the clustering node to produce clusters, bounding boxes and `DetectedObjectArray`
    ```sh
    . install/setup.bash
    ros2 run project_tracker cluster
    ```
8.  In a new terminal, call the mock image publisher node to publish images to the /camera topic. this node takes two arguments from the command line. `Image_path` and `Frame_Id`
    ```sh
    . install/setub.bash
    ros2 run project_tracker mock_image_pub.py <Image_Path> <Frame_Id>
    eg. ros2 run project_tracker mock_image_pub.py /home/mcav/DATASETS/streetViewImages/ velodyne 
    ```
9. In a new terminal, call the object detection node to detect objects from the images published to the /camera topic.
    ```sh
    . install/setub.bash
    ros2 run project_tracker object_detection.py
    ```

#### Parameters and ROS Info

##### Parameters to modify for clustering: 
* `self.min_cluster_size`: minimum number of LiDAR points to be considered a cluster
* `self.max_cluster_size`: maximum number of LiDAR points to be considered a cluster
* `self.cluster_tolerance`: unsure exactly what the interpretation of this parameter is. I originally thought it was the maximum search radius from any point to another but think it is more nuanced than that after playing with it.

##### Topics
|Topic|Type|Objective|Nodes interacting|
------|----|---------|-----------------|
|`/velodyne_points`|`sensor_msgs.msg PCL2`|Publish mock LiDAR data|`project_tracker::mock_pub` publishes, `project_tracker::filter` subscribes|
|`clustered_pointclouds`|`sensor_msgs.msg PCL2`|View clustered pointclouds in PCL2 format for visualisation|`project_tracker::cluster` publishes|
|`bounding_boxes`|`visualization_msgs.msg MarkerArray`|View bounding boxes over identified clusters for visualisation|`project_tracker::cluster` publishes|
|`detected_objects`|`mcav_interfaces::DetectedObjectArray`|Emit detected objects for use in other MCAV nodes e.g. path planning|`project_tracker::cluster` publishes|
|`/camera`|`sensor_msgs.msg Image`|publish mock Images|`project_tracker::mock_image_pub` publishes, `project_tracker::object_detection` subscribes|

## Carla Integration

#### Recording ROS bags in Carla

Manual Control Workaround to Autopilot issue

1. Start Carla Agent `/opt/carla-simulator/CarlaUE4.sh -quality-level=Low`
2. Open a new terminal, source carla_ros_bridge
```bash
cd <PATH-TO-carla_ros_bridge> (on the beauty this is ~/Sheng/carla_ros_bridge, the beast it is ~/liam_ws/carla_ros_bridge, I think)
source  ./install/setup.bash
```
<<<<<<< HEAD
3. Launch carla_ros_bridge `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py`
4. Launch carla_spawn_npc `ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py objects_definition_file:='<PATH-TO-project_tracker/carla_integration>/tracking.json'`
=======
3. Launch carla_ros_bridge `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py -timeout:=10`
4. Open a new terminal, launch carla_spawn_npc `ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py objects_definition_file:='./tracking.json'`
>>>>>>> carla2
    Must make sure to modify the `'objects_definition_file'` in carla_ros_bridge.launch.py to reflect where .json objects file is stored
5. Open a new terminal, launch carla_manual_control `ros2 launch carla_manual_control carla_manual_control.launch.py`
6. Open a new terminal, spawn vehicles and walkers `python3 ./carla_integration/generate_traffic.py -n 150 -w 100 --no-rendering`
7. Drive the car using this manual control window
8. Open rviz2 in a new terminal and set frame_id to `ego_vehicle`, add pointcloud from `/carla/ego_vehicle/lidar` and camera from `/carla/ego_vehicle/rgb_front`
9. Optionally record ROS bags for later use in a new terminal
```bash
ros2 bag record -o <TOPIC-NAME> `ros2 topic list | grep --regexp="/carla/*"` /tf
```

#### Playing back ROS bags with tracking

1. Play ROS bags back (at faster rate as recording lags a lot)
```bash
cd <PROJECT_TRACKER-PACKAGE>/bag_files
ros2 bag play <BAG-NAME> -r 2.0
```
2. Open a new terminal, source workspace setup file and run tracking_carla launch file
```bash
cd <WORKSPACE> 
. install/setup.bash
ros2 launch project_tracker tracking_carla.launch.py
```
3. Open a new terminal, launch rviz2 and set frame_id to 'velodyne'. Add relevant pointcloud/image topics


## TODO

### Carla ROS bags with self-driving ego_vehicle and autopilot traffic

Need to figure out how to get auto-pilot cars/pedestrians in that don't crash (Traffic Manager issue)

* Think issue stems from fact that traffic manager that is created with carla_ad_demo overrides the traffic manager created when spawning extra traffic with generate_traffic.py from PythonAPI/examples. Link below may help solve issue, but unsure whether anything is being done wrong
https://carla.readthedocs.io/en/latest/adv_traffic_manager/

**Steps**

1. Start Carla Agent `/opt/carla-simulator/CarlaUE4.sh`
2. Source carla_ad_demo repo
```bash
cd <PATH-TO-carla_ros_bridge> (on the beauty this is ~/Sheng/carla_ros_bridge, the beast it is ~/liam_ws/carla_ros_bridge, I think)
source  ./install/setup.bash
```
3. Launch carla_ad_demo `ros2 launch carla_ad_demo carla_ad_demo.launch.py`
    Must make sure to modify the `'objects_definition_file'` in carla_ad_demo.launch.py to reflect where .json objects file is stored
4. **TODO:** Spawn NPCs -> currently they crash as soon as they are spawned which makes it impossible for the AD to drive
5. Open rviz and set frame_id to `ego_vehicle`, add pointcloud from `/carla/ego_vehicle/lidar` and camera from `/carla/ego_vehicle/rgb_front`
6. Optionally record ROS bags for later use
```bash
ros2 bag record -o <TOPIC-NAME> `ros2 topic list | grep --regexp="/carla/*"` /tf
```



## Contact
Amir Toosi - amir.ghalb@gmail.com

Ben Edwards - bedw0004@student.monash.edu
