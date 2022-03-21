# Project Tracker

Project tracker takes inputs from the @Multi-Task Panoptic Perception model and LiDAR sensor and fuses them together to create accurate pose and velocity estimates of the object over time in 3D space.

![tracker](https://user-images.githubusercontent.com/69286161/151936865-c160a7b6-f4cc-4b03-b0d2-b586d1aff493.gif)



## Getting Started

### prerequisites
* create a ros2 workspace by following [these](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) instructions
* clone this repository into the src directory in your ros2 workspace

### Installation

1. Download synced+rectified data from [here](http://www.cvlibs.net/datasets/kitti/raw_data.php) or to download the data directly, press [this](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0048/2011_09_26_drive_0048_sync.zip) download button
2. After the download is complete, navigate to ```velodyne_points/data``` then copy-and-paste the binary files to ```data/velodyne_points``` in the project folders
3. navigate to the root folder of your workspace
4. build the project
	```sh
    colcon build
    . install/setup.bash
    ```
    
5. in a new terminal, call the publisher
	```sh
    . install/setup.bash
    ros2 run project_tracker mock_pub
    ```
    
6.  in a new terminal, call the subscriber
    ```sh
    . install/setup.bash
    ros2 run project_tracker mock_sub
    ```

#### Clustering instructions

```bash
sudo apt install python3-pcl
sudo apt-get install ros-galactic-sensor-msgs-py
```

* Parameters to modify: self.min_cluster_size, self.max_cluster_size, self.max_search_radius
* Subscribing to `pcl2conversion`
* Publishing to `clustered_pointclouds` -> publishes coloured pointclouds to show clusters


## Contact
Amir Toosi - amir.ghalb@gmail.com

Ben Edwards - bedw0004@student.monash.edu
