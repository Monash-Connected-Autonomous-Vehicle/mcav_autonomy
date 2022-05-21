# :cloud: Project Tracker

Project tracker takes inputs from the @Multi-Task Panoptic Perception model and LiDAR sensor and fuses them together to create accurate pose and velocity estimates of the object over time in 3D space.

[![All Contributors](https://img.shields.io/badge/all_contributors-1-orange.svg?style=flat-square)](#contributors)

![tracker](https://user-images.githubusercontent.com/69286161/151936865-c160a7b6-f4cc-4b03-b0d2-b586d1aff493.gif)

## Requirements
- [Ubuntu 20.04](https://ubuntu.com/download/desktop) or [WSL on Windows 10/11](https://docs.microsoft.com/en-us/windows/wsl/install)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- Download synced and rectified KITTI data [here](http://www.cvlibs.net/datasets/kitti/raw_data.php) or to download the data directly, press [here](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0048/2011_09_26_drive_0048_sync.zip)
- Install dependencies
    ```sh
    sudo apt install python3-pcl
    sudo apt-get install ros-galactic-sensor-msgs-py
    sudo apt-get install ros-galactic-tf-transformations
    sudo pip3 install transforms3d
    ```

### Requirements for CARLA Example
- [CARLA](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [CARLA ROS2 Bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/)

## Installation

### Moving KITTI data to correct directory

* Move download of KITTI dataset to ```/home/mcav/DATASETS/KITTI/```, unzip the downloaded zip and copy the folder named `2011_09_26` into the `KITTI` directory.  

```
home/mcav/DATASETS/                                                     
├── KITTI                                                                                                               
    └── 2011_09_26
        └── 2011_09_25_drive_0048_sync
            ├── image_00                  # image data and timestamps
                ├── data
            ├── image_01                  # image data and timestamps
                ├── data
            ├── image_02                  # image data and timestamps
                ├── data
            ├── image_03                  # image data and timestamps
                ├── data
            ├── oxts                      # IMU data and timestamps
                ├── data
            └── velodyne_points           # LiDAR pointcloud data and timestamps
                ├── data
```

Alternatively move your KITTI folder into another folder and specify this when calling `mock_pub.py` with ROS arguments:
```sh
ros2 run project_tracker mock_pub.py --ros-args -p kitti_data_dir:="PATH_TO_YOUR_KITTI/2011_09_26/2011_09_25_drive_0048_sync_DIRECTORY"
```

### Creating workspace and package

* Create a ROS2 workspace by following [these instructions](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) 
* Go to the source directory: `cd ~/mcav_ws/src`
* Clone this repository: `git clone`
* Go to the root of the workspace: `cd ~/mcav_ws`
* Install ROS dependencies: `rosdep install --from-paths src --ignore-src -r -y`
* Build the package: `colcon build`

This should result in a directory structre similar to the following:
```
mcav_ws/                                                     
├── build   
├── install
├── log                                                                                                              
└── src
    └── project_tracker
        ├── carla_integration       # files relevant to running CARLA example
        ├── launch                  # launch files for different examples
        ├── project_tracker         # python scripts used in the package
        └── src                     # C++ nodes used in the package
```

## Usage

Note, for every terminal opened you should navigate to the root folder of your workspace (`cd ~/mcav_ws`) and source the setup file (`. install/setup.bash`).

### KITTI Example

Terminal 1 (Mock KITTI Publisher):
```sh
ros2 run project_tracker mock_pub.py --ros-args -p kitti_data_dir:="PATH_TO_YOUR_KITTI/2011_09_26/2011_09_25_drive_0048_sync_DIRECTORY"
```

Terminal 2 (Filter Node to reduce number of LiDAR points):
```sh
ros2 run project_tracker filter
```

Terminal 3 (Cluster Node to produce clusters, bounding boxes and `DetectedObjectArray`):
```sh
ros2 run project_tracker cluster.py
```

Terminal 4 (Mock Image Publisher to publish images):
```sh
ros2 run project_tracker mock_image_pub.py <Image_Path> <Frame_Id>
eg. ros2 run project_tracker mock_image_pub.py /home/mcav/DATASETS/streetViewImages/ velodyne 
```

Terminal 5 (Object Detection Node to detect objects from images):
```sh
ros2 run project_tracker object_detection.py
```

### CARLA Example

#### Recording ROS bags in Carla

Terminal 1 (Launch CARLA Server):
```sh
/opt/carla-simulator/CarlaUE4.sh -quality-level=Low
```

Terminal 2 (Source and launch CARLA ROS Bridge):
```sh
cd <PATH-TO-carla_ros_bridge> 
source  ./install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py -timeout:=10
```

Terminal 3 (Spawn Ego Vehicle):
Must make sure to modify the `'objects_definition_file'` in carla_ros_bridge.launch.py to reflect where .json objects file is stored
```sh
ros2 launch carla_spawn_objects carla_example_ego_vehicle.launch.py objects_definition_file:='./carla_integration/tracking.json'
```

Terminal 4 (Take manual control of Ego Vehicle):
```sh
ros2 launch carla_manual_control carla_manual_control.launch.py
```

Terminal 5 (Spawn Non-Player Characters: vehicles and pedestrians):
Set to no rendering mode to reduce computational load in non-manual control window.
```sh
python3 ./carla_integration/generate_traffic.py -n 150 -w 100 --no-rendering
```

* Drive the car using the manual control window
* Open rviz2 in a new terminal and set frame_id to `ego_vehicle`, add pointcloud from `/carla/ego_vehicle/lidar` and camera from `/carla/ego_vehicle/rgb_front`

#### Optional: record and play back ROS Bags from CARLA
**While driving with manual control:**
* New terminal: record ROS bags for later use
```bash
cd ~/mcav_ws/src/project_tracker/bag_files
ros2 bag record -o <RECORD-DIR> `ros2 topic list | grep --regexp="/carla/*"` /tf
e.g. ros2 bag record -o manual_150 `ros2 topic list | grep --regexp="/carla/*"` /tf
```
**Playing back ROS bags later**:
* Terminal 1: play ROS bags back (at faster rate as recording lags a lot)
```bash
cd ~/mcav_ws/src/project_tracker/bag_files
ros2 bag play <BAG-NAME> -r 2.0
e.g. ros2 bag play manual_150 -r 2.0
```
* Terminal 2: source workspace setup file and run tracking_carla launch file
```bash
cd ~/mcav_ws
. install/setup.bash
ros2 launch project_tracker tracking_carla.launch.py
```
* Terminal 3: launch rviz2 and set frame_id to 'velodyne'. Add relevant pointcloud/image topics


## ROS Parameters and Topics
Please see the [`ROSINFO.md`](https://github.com/Monash-Connected-Autonomous-Vehicle/mcav-GitHub-documentation-standard/blob/main/ROSINFO.md) file for more info.

## Contributors ✨

Thanks goes to these people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore -->
<table>
    <tr>
        <td align="center"><a href="https://github.com/bened-wards"><img src="https://avatars.githubusercontent.com/u/69286161?v=4" width="100px;" alt="Ben Edwards"/><br /><sub><b>Ben Edwards</b></sub></a><br /><a title="Code">💻</a></td>
        <td align="center"><a href="https://github.com/amir-kt"><img src="https://avatars.githubusercontent.com/u/54131619?v=4" width="100px;" alt="Amir Toosi"/><br /><sub><b>Amir Toosi</b></sub></a><br /><a title="Code">💻</a></td>
        <td align="center"><a href="https://github.com/lakshjaisinghani"><img src="https://avatars3.githubusercontent.com/u/45281017?v=4" width="100px;" alt="Laksh Jaisinghani"/><br /><sub><b>Laksh Jaisinghani</b></sub></a><br /><a title="Mentoring">🧑‍🏫 </a></td>
        <td align="center"><a href="https://github.com/owenbrooks"><img src="https://avatars.githubusercontent.com/u/7232997?v=4" width="100px;" alt="Owen Brooks"/><br /><sub><b>Owen Brooks</b></sub></a><br /><a title="Review">👀 </a></td>
    </tr>
</table>

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!


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
