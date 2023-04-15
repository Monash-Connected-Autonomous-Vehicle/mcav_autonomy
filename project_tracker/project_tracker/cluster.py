#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import logging
import time
from math import sqrt

import numpy as np

from sensor_msgs.msg import PointCloud2 as PCL2
from visualization_msgs.msg import MarkerArray, Marker
from transforms3d.quaternions import mat2quat
from transforms3d.euler import mat2euler, euler2quat

from utils import numpy_2_PCL2, PCL2_2_numpy, create_colour_list, pcl_to_ros
from tracking import Tracker

from mcav_interfaces.msg import DetectedObject, DetectedObjectArray

import pcl

import config

class PCL2Subscriber(Node):
    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            PCL2,
            '/velodyne_filtered',
            self.points_callback,
            10
        )

        self.pointcloud: pcl.PointCloud
        self.np_pointcloud: np.ndarray
        self.original_frame_id: str

        # TODO create cloud cluster publisher via creating a custom msg
        self._cloud_cluster_publisher = self.create_publisher(PCL2, 'clustered_pointclouds', 10)
        self._bounding_boxes_publisher = self.create_publisher(MarkerArray, 'bounding_boxes_array', 10)
        self._detected_objects_publisher = self.create_publisher(DetectedObjectArray, 'detected_objects', 10)
        
        # create tracker for identifying and following objects over time
        # self.tracker = Tracker(max_frames_before_forget=2, max_frames_length=30, tracking_method="centre_distance", dist_threshold=5)
        # self.tracker = Tracker(max_frames_before_forget=2, max_frames_length=30, tracking_method="iou", iou_threshold=0.85)
        self.tracker = Tracker(
            max_frames_before_forget=2, 
            max_frames_length=30, 
            tracking_method="both", 
            iou_threshold=0.85,
            dist_threshold = 5
        )

        # colour list for publishing different clusters
        self.rgb_list, self.colour_list = create_colour_list()

        self.get_logger().set_level(logging.DEBUG)


    def points_callback(self, msg):
        """Receive PCL2 message and convert it to points."""
        # create and save a pythonpcl pointcloud and its numpy representation from the ros message
        self.np_pointcloud, self.pointcloud = self.load_pointcloud_from_ros_msg(msg)

        # make kdtree from the pointcloud
        kd_tree = self.pointcloud.make_kdtree()

        # a list of indices representing each set of points in np_pointcloud that are in the same cluster
        np_pointcloud_cluster_indices = self.create_euclidean_cluster(
            self.pointcloud, kd_tree, config.cluster_tolerance, config.min_cluster_size, config.max_cluster_size)

        # a list representing a coloured version of the clusters in the pointcloud for visualisation
        coloured_clustered_points = self.create_coloured_pointcloud_clusters(np_pointcloud_cluster_indices)

        # convert to pcl.PointCloud_PointXYZRGB for visualisation in RViz
        coloured_clustered_pointcloud = pcl.PointCloud_PointXYZRGB()
        coloured_clustered_pointcloud.from_list(coloured_clustered_points)
        timestamp = self.get_clock().now().to_msg()

        # convert the pcl to a ROS PCL2 message
        pcl2_msg = pcl_to_ros(coloured_clustered_pointcloud,
                              timestamp, self.original_frame_id)

        self._cloud_cluster_publisher.publish(pcl2_msg)
        
        # fit bounding boxes to the clustered pointclouds
        detected_objects = self.create_detected_objects(np_pointcloud_cluster_indices) 

        # track objects over time
        tracked_detected_objects = self.tracker.update(detected_objects)
        # self.get_logger().info(f'Number of tracked objects: {len(tracked_detected_objects.detected_objects)}')
        # print(f"Number of tracked objects: {len(tracked_detected_objects.detected_objects)}")

        # fit bounding boxes and ID labels
        self.fit_bounding_boxes(tracked_detected_objects)

        # publish bounding boxes and detected objects
        self._bounding_boxes_publisher.publish(self.markers)
        self._detected_objects_publisher.publish(tracked_detected_objects)


    def create_coloured_pointcloud_clusters(self, np_pointcloud_cluster_indices):
        # colouring the clouds
        coloured_clustered_points = []
        for j, indices in enumerate(np_pointcloud_cluster_indices):
            for idx in indices:
                coloured_clustered_points.append([
                    self.np_pointcloud[idx][0],
                    self.np_pointcloud[idx][1],
                    self.np_pointcloud[idx][2],
                    self.colour_list[j]
                ])
        return coloured_clustered_points


    def load_pointcloud_from_ros_msg(self, msg):
        self.original_frame_id = msg.header.frame_id
        np_full_pointcloud = PCL2_2_numpy(msg, reflectance=False)
        np_pointcloud = np_full_pointcloud[:, :3] # ignore reflectance for clustering

        ## python3-pcl binding euclidean clustering
        pointcloud = pcl.PointCloud() # create empty pcl.PointCloud to use C++ bindings to PCL 
        pointcloud.from_array(np_pointcloud)
        
        return np_pointcloud, pointcloud


    def create_euclidean_cluster(self, pointcloud, kd_tree, cluster_tolerance, min_cluster_size, max_cluster_size):
        """
        Perform euclidean clustering with a given pcl.PointCloud() and kdtree.

        Parameters
        ----------
        pointcloud : pcl.PointCloud()
            pcl version of pointcloud message received
        kd_tree : kdtree
            kdtree from pcl-python binding
        cluster_tolerance: float 
            distance below which points are considered part of a cluster
        min_cluster_size: int
            minimum size of a cluster
        max_cluster_size: int
            maximum size of a cluster

        """
        # make euclidean cluster extraction method
        ec = pointcloud.make_EuclideanClusterExtraction()
        # set parameters
        ec.set_ClusterTolerance(cluster_tolerance)
        ec.set_MinClusterSize(min_cluster_size + 1)
        ec.set_MaxClusterSize(max_cluster_size)
        ec.set_SearchMethod(kd_tree)

        # perform euclidean clustering and return indices
        return ec.Extract()


    def create_detected_objects(self, np_pointcloud_cluster_indices):
        """
        Create detected objects from the clusters by finding their centre points and dimensions.
        
        This creates the constraints necessary to fit a bounding box later.
        
        Tutorial at PCL docs helps with make_MomentOfInertiaEstimation aspect
        https://pcl.readthedocs.io/projects/tutorials/en/master/moment_of_inertia.html#moment-of-inertia
        """
        # self.get_logger().info(f'number of clusters: {len(np_pointcloud_cluster_indices)}')
        objects = DetectedObjectArray()

        for cluster_idx, indices in enumerate(np_pointcloud_cluster_indices):
            cluster_cloud = self.np_pointcloud[list(indices)] # numpy array cloud
            # Create flattened point cloud by zeroing z values.
            # bounding box orientations are found in this 2D representation since it gave nicer results.
            # The axis-aligned bounding box of the origin 3D cloud is also used, but only for the height and z-axis position.
            # Would be worth experimenting with pure 3D boxes again in future.
            flattened_cloud = cluster_cloud.copy()
            flattened_cloud[:, 2] = 0.0 # set z values to 0 to flatten cloud.
            
            # convert to pcl object
            cluster_pcl_cloud = pcl.PointCloud()
            cluster_pcl_cloud.from_array(cluster_cloud) 

            flattened_pcl_cloud = pcl.PointCloud()
            flattened_pcl_cloud.from_array(flattened_cloud) 

            # create feature extractor for axis-aligned bounding box
            feature_extractor_3d = cluster_pcl_cloud.make_MomentOfInertiaEstimation()
            feature_extractor_3d.compute()
            # Axis-Aligned Bounding Box (AABB)
            [min_point_AABB, max_point_AABB] = feature_extractor_3d.get_AABB()

            # create feature extractor for flattened bounding box
            feature_extractor_flat = flattened_pcl_cloud.make_MomentOfInertiaEstimation()
            feature_extractor_flat.compute()
            # Oriented Bounding Box (OBB)
            [_, max_point_OBB, position_OBB, rotational_matrix_OBB] = feature_extractor_flat.get_OBB()

            # create detected object
            detected_object = DetectedObject()
            detected_object.object_id = cluster_idx # dummy value until we track the objects
            detected_object.frame_id = self.original_frame_id

            # convert rotational matrix to quaternion for use in pose
            roll, pitch, yaw = mat2euler(rotational_matrix_OBB)
            quat = euler2quat(roll, pitch, yaw, 'sxyz')
            detected_object.pose.orientation.w = quat[0]
            detected_object.pose.orientation.x = quat[1]
            detected_object.pose.orientation.y = quat[2]
            detected_object.pose.orientation.z = quat[3]

            # pose -> assume of center point
            z_pos = (max_point_AABB[0, 2] + min_point_AABB[0, 2])/2
            detected_object.pose.position.x = float(position_OBB[0,0])
            detected_object.pose.position.y = float(position_OBB[0,1])
            detected_object.pose.position.z = float(z_pos)

            # dimensions -> assuming want distance from face to face
            detected_object.dimensions.x = 2 * float(max_point_OBB[0, 0])
            detected_object.dimensions.y = 2 * float(max_point_OBB[0, 1])
            detected_object.dimensions.z = float(max_point_AABB[0, 2] - min_point_AABB[0, 2])
            
            # object and signal class -> unknown for now
            detected_object.object_class = detected_object.CLASS_UNKNOWN
            detected_object.signal_state = detected_object.SIGNAL_UNKNOWN

            objects.detected_objects.append(detected_object)

        # self.get_logger().info(f'number of objects: {len(objects.detected_objects)}')
        return objects


    def fit_bounding_boxes(self, tracked_detected_objects):
        """Fit bounding boxes to tracked detected objects."""
        self.markers = MarkerArray() # list of markers for visualisations of boxes/IDs

        for d_o in tracked_detected_objects.detected_objects:
            id_marker = Marker()
            id_marker.ns = 'object_id'
            id_marker.id = d_o.object_id
            id_marker.header.frame_id = self.original_frame_id
            id_marker.type = Marker.TEXT_VIEW_FACING
            id_marker.action = id_marker.ADD

            # Keeping original bounding box labelling
            id_marker.pose.position.x = d_o.pose.position.x + 0.5
            id_marker.pose.position.y = d_o.pose.position.y
            id_marker.pose.position.z = d_o.pose.position.z
            id_marker.color.a = 1.
            id_marker.color.r = 1.
            id_marker.color.g = 1.
            id_marker.color.b = 1.
            id_marker.scale.x = 1.
            id_marker.scale.y = 0.8
            id_marker.scale.z = 0.5
            id_marker.text = f"{d_o.object_id}"
            self.markers.markers.append(id_marker)

            # create bounding boxes for visualisation
            bounding_box_marker = Marker()
            bounding_box_marker.ns = 'bounding_boxes_array'
            bounding_box_marker.id = d_o.object_id
            bounding_box_marker.header.frame_id = self.original_frame_id
            bounding_box_marker.type = Marker.CUBE
            bounding_box_marker.action = Marker.ADD
            bounding_box_marker.color.a = 0.5
            bounding_box_marker.color.r = 255.#self.rgb_list[cluster_idx][0]/255.
            bounding_box_marker.color.g = 255.#self.rgb_list[cluster_idx][1]/255.
            bounding_box_marker.color.b = 255.#self.rgb_list[cluster_idx][2]/255.

            bounding_box_marker.scale.x = d_o.dimensions.x
            bounding_box_marker.scale.y = d_o.dimensions.y
            bounding_box_marker.scale.z = d_o.dimensions.z
            bounding_box_marker.pose = d_o.pose
            self.markers.markers.append(bounding_box_marker)

        try:
            # delete bounding boxes and IDs that aren't present in this frame but were in previous
            for delete_id in self.tracker.deleted_ids:
                del_id_marker = Marker()
                del_id_marker.ns = 'object_id'
                del_id_marker.header.frame_id = self.original_frame_id
                del_id_marker.id = delete_id
                del_id_marker.action = Marker.DELETE
                self.markers.markers.append(del_id_marker)
                del_bb_marker = Marker()
                del_bb_marker.ns = 'bounding_boxes_array'
                del_bb_marker.header.frame_id = self.original_frame_id
                del_bb_marker.id = delete_id
                del_bb_marker.action = Marker.DELETE
                self.markers.markers.append(del_bb_marker)
        except AttributeError: # if only 1 frame there are no bounding boxes to delete
            pass



def main(args=None):
    rclpy.init(args=args)

    pcl2_subscriber = PCL2Subscriber()
    
    try:
        rclpy.spin(pcl2_subscriber)
    except KeyboardInterrupt:
        pcl2_subscriber.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    pcl2_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()