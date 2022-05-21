# Parameters and ROS Info

## Parameters to modify for clustering: 
* `self.min_cluster_size`: minimum number of LiDAR points to be considered a cluster
* `self.max_cluster_size`: maximum number of LiDAR points to be considered a cluster
* `self.cluster_tolerance`: unsure exactly what the interpretation of this parameter is. I originally thought it was the maximum search radius from any point to another but think it is more nuanced than that after playing with it.

## Topics
|Topic|Type|Objective|Nodes interacting|
------|----|---------|-----------------|
|`/velodyne_points`|`sensor_msgs.msg PCL2`|Publish mock LiDAR data|`project_tracker::mock_pub` publishes, `project_tracker::filter` subscribes|
|`clustered_pointclouds`|`sensor_msgs.msg PCL2`|View clustered pointclouds in PCL2 format for visualisation|`project_tracker::cluster` publishes|
|`bounding_boxes`|`visualization_msgs.msg MarkerArray`|View bounding boxes over identified clusters for visualisation|`project_tracker::cluster` publishes|
|`detected_objects`|`mcav_interfaces::DetectedObjectArray`|Emit detected objects for use in other MCAV nodes e.g. path planning|`project_tracker::cluster` publishes|
|`/camera`|`sensor_msgs.msg Image`|publish mock Images|`project_tracker::mock_image_pub` publishes, `project_tracker::object_detection` subscribes|