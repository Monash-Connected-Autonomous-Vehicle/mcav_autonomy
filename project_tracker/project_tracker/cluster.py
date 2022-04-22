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

from project_tracker.utils import numpy_2_PCL2, PCL2_2_numpy, create_colour_list, pcl_to_ros
from project_tracker.tracking import Tracker

from mcav_interfaces.msg import DetectedObject, DetectedObjectArray

import pcl


class PCL2Subscriber(Node):

    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            PCL2,
            '/velodyne_filtered',
            self._callback,
            10
        )

        # TODO create cloud cluster publisher via creating a custom msg
        self._cloud_cluster_publisher = self.create_publisher(PCL2, 'clustered_pointclouds', 10)
        self._bounding_boxes_publisher = self.create_publisher(MarkerArray, 'bounding_boxes', 10)
        self._detected_objects_publisher = self.create_publisher(DetectedObjectArray, 'detected_objects', 10)
        

        # parameters for Euclidean Clustering
        self.min_cluster_size = 10
        self.max_cluster_size = 600
        self.cluster_tolerance = 0.4 # range from 0.5 -> 0.7 seems suitable. Test more when have more data

        # create tracker for identifying and following objects over time
        # self.tracker = Tracker(max_frames_before_forget=2, max_frames_length=30, tracking_method="centre_distance", dist_threshold=5)
        # self.tracker = Tracker(max_frames_before_forget=2, max_frames_length=30, tracking_method="iou", iou_threshold=0.85)
        self.tracker = Tracker(
            max_frames_before_forget=2, max_frames_length=30, tracking_method="both", 
            iou_threshold=0.85, dist_threshold = 5
        )

        # colour list for publishing different clusters
        self.rgb_list, self.colour_list = create_colour_list()

        self.get_logger().set_level(logging.DEBUG)


    def _callback(self, msg):
        """Subscriber callback. Receives PCL2 message and converts it to points"""
        start = time.time()
        self.original_frame_id = msg.header.frame_id
        self.full_cloud = PCL2_2_numpy(msg, reflectance=False)
        self.cloud = self.full_cloud[:, :3] # ignore reflectance for clustering
        
        self.no_samples, self.no_axes = self.cloud.shape
        cloud_time = time.time() - start
        self.get_logger().info(f"Took {cloud_time:.5f}s to receive pcl2 message: {self.no_samples, self.no_axes}")

        ## python3-pcl binding euclidean clustering
        ec_cloud = pcl.PointCloud() # create empty pcl.PointCloud to use C++ bindings to PCL 
        ec_cloud.from_array(self.cloud)
        ec_tree = ec_cloud.make_kdtree() # make kdtree from our tree
        start = time.time()
        self.euclidean_clustering_ec(ec_cloud, ec_tree)
        clustering_ec_time = time.time() - start
        self.get_logger().info(f"PCL binding ec took {clustering_ec_time:.5f}s to find {len(self.clusters_ec)} clusters.")

        ## PUBLISHING
        # COLOURED VERSION
        # colouring the clouds
        self.colour_cluster_point_list = []
        for j, indices in enumerate(self.clusters_ec):
            for indice in indices:
                self.colour_cluster_point_list.append([
                    self.cloud[indice][0],
                    self.cloud[indice][1],
                    self.cloud[indice][2],
                    self.colour_list[j]
                ])
                
        # convert to pcl.PointCloud_PointXYZRGB for visualisation in RViz
        cluster_colour_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_colour_cloud.from_list(self.colour_cluster_point_list)
        timestamp = self.get_clock().now().to_msg()
        pcl2_msg = pcl_to_ros(cluster_colour_cloud, timestamp, self.original_frame_id) # convert the pcl to a ROS PCL2 message
        self._cloud_cluster_publisher.publish(pcl2_msg)

        # fit bounding boxes to the clustered pointclouds
        start = time.time()
        detected_objects = self.create_detected_objects() 

        # track objects over time
        tracked_detected_objects = self.tracker.update(detected_objects)
        print(f"Number of tracked objects: {len(tracked_detected_objects.detected_objects)}")

        # fit bounding boxes and ID labels
        self.fit_bounding_boxes(tracked_detected_objects)

        tracking_time = time.time() - start
        self.get_logger().info(f"Tracking took {tracking_time:.5f}s")

        # publish bounding boxes and detected objects
        self._bounding_boxes_publisher.publish(self.markers)
        self._detected_objects_publisher.publish(tracked_detected_objects)

    def euclidean_clustering_ec(self, ec_cloud, ec_tree):
        """
        Perform euclidean clustering with a given pcl.PointCloud() and kdtree

        Parameters
        ----------
        ec_cloud : pcl.PointCloud()
            pcl version of pointcloud message received
        ec_tree : kdtree
            kdtree from pcl-python binding
        """
        # make euclidean cluster extraction method
        ec = ec_cloud.make_EuclideanClusterExtraction()
        # set parameters
        ec.set_ClusterTolerance(self.cluster_tolerance)
        ec.set_MinClusterSize(self.min_cluster_size + 1)
        ec.set_MaxClusterSize(self.max_cluster_size)
        ec.set_SearchMethod(ec_tree)
        # perform euclidean clustering and return indices
        self.clusters_ec = ec.Extract()
        return

    def create_detected_objects(self):
        """
        Create detected objects from the clusters by finding their centre points and dimensions. This 
        creates the constraints necessary to fit a bounding box later.
        
        Tutorial at PCL docs helps with make_MomentOfInertiaEstimation aspect
        https://pcl.readthedocs.io/projects/tutorials/en/master/moment_of_inertia.html#moment-of-inertia
        """
        objects = DetectedObjectArray()

        for cluster_idx, indices in enumerate(self.clusters_ec):
            cloud = self.cloud[list(indices)] # numpy array cloud
            # convert to pcl object
            bb_cloud = pcl.PointCloud()
            bb_cloud.from_array(cloud) 

            # create feature extractor for bounding box
            feature_extractor = bb_cloud.make_MomentOfInertiaEstimation()
            feature_extractor.compute()
            # axis-aligned bounding box
            [min_point_AABB, max_point_AABB] = feature_extractor.get_AABB()
            # oriented bounding box
            [min_point_OBB, max_point_OBB, position_OBB,
                rotational_matrix_OBB] = feature_extractor.get_OBB()

            # create detected object
            detected_object = DetectedObject()
            detected_object.object_id = cluster_idx # dummy value until we track the objects
            detected_object.frame_id = self.original_frame_id

            # convert rotational matrix to quaternion for use in pose
            roll, pitch, yaw = mat2euler(rotational_matrix_OBB)
            while not(-10. < yaw*180/np.pi < 10.):
                yaw -= np.sign(yaw) * 0.15
            quat = euler2quat(0., 0., yaw)
            # pose -> assume of center point
            detected_object.pose.orientation.w = quat[0]
            detected_object.pose.orientation.x = quat[1]
            detected_object.pose.orientation.y = quat[2]
            detected_object.pose.orientation.z = quat[3]
            # # orientation -> restricted to rotate only around the z axis i.e. flat to ground plane
            # mag = sqrt(quat[0]**2 + quat[3]**2)
            # detected_object.pose.orientation.w = float(quat[0]/mag)
            # detected_object.pose.orientation.x = 0. #float(quat[1])
            # detected_object.pose.orientation.y = 0. #float(quat[2]/mag)
            # detected_object.pose.orientation.z = float(quat[2]/mag)#float(quat[3])

            ### oriented version
            detected_object.pose.position.x = float(position_OBB[0,0])
            detected_object.pose.position.y = float(position_OBB[0,1])
            detected_object.pose.position.z = float(position_OBB[0,2])
            # dimensions -> assuming want distance from face to face
            detected_object.dimensions.x = 2 * float(max_point_OBB[0,0])
            detected_object.dimensions.y = 2 * float(max_point_OBB[0,1])
            detected_object.dimensions.z = 2 * float(max_point_OBB[0,2])
            # object and signal class -> unknown for now
            detected_object.object_class = detected_object.CLASS_UNKNOWN
            detected_object.signal_state = detected_object.SIGNAL_UNKNOWN

            objects.detected_objects.append(detected_object)

        return objects

    def fit_bounding_boxes(self, tracked_detected_objects):
        """
        Fit bounding boxes to tracked detected objects
        """
        self.markers = MarkerArray() # list of markers for visualisations of boxes/IDs

        for d_o in tracked_detected_objects.detected_objects:
            id_marker = Marker()
            id_marker.ns = 'object_id'
            id_marker.id = d_o.object_id
            id_marker.header.frame_id = self.original_frame_id
            id_marker.type = Marker.TEXT_VIEW_FACING
            id_marker.action = id_marker.ADD
            id_marker.pose.position.x = d_o.pose.position.x
            id_marker.pose.position.y = d_o.pose.position.y + 0.5
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
            bounding_box_marker.ns = 'bounding_boxes'
            bounding_box_marker.id = d_o.object_id
            bounding_box_marker.header.frame_id = self.original_frame_id
            bounding_box_marker.type = Marker.CUBE
            bounding_box_marker.action = Marker.ADD
            bounding_box_marker.color.a = 0.5
            bounding_box_marker.color.r = 255.#self.rgb_list[cluster_idx][0]/255.
            bounding_box_marker.color.g = 255.#self.rgb_list[cluster_idx][1]/255.
            bounding_box_marker.color.b = 255.#self.rgb_list[cluster_idx][2]/255.
            # size -> 2 times the max_point from centre
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
                del_bb_marker.ns = 'bounding_boxes'
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