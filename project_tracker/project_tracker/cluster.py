import rclpy
from rclpy.node import Node

import logging
import time
from math import sqrt

from sensor_msgs.msg import PointCloud2 as PCL2
from visualization_msgs.msg import MarkerArray, Marker
from transforms3d.quaternions import mat2quat

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
        self.tracker = Tracker(dist_threshold=4, max_frames_before_forget=2, max_frames_length=30)

        # colour list for publishing different clusters
        self.rgb_list, self.colour_list = create_colour_list()

        self.get_logger().set_level(logging.DEBUG)


    def _callback(self, msg):
        """Subscriber callback. Receives PCL2 message and converts it to points"""
        start = time.time()

        self.full_cloud = PCL2_2_numpy(msg, reflectance=False)
        self.cloud = self.full_cloud[:, :3] # ignore reflectance for clustering
        
        self.no_samples, self.no_axes = self.cloud.shape
        cloud_time = time.time() - start
        self.get_logger().info(f"Took {cloud_time:.5f} to receive pcl2 message: {self.no_samples, self.no_axes}\n{self.cloud[0]}\n")

        ## python3-pcl binding euclidean clustering
        ec_cloud = pcl.PointCloud() # create empty pcl.PointCloud to use C++ bindings to PCL 
        ec_cloud.from_array(self.cloud)
        ec_tree = ec_cloud.make_kdtree() # make kdtree from our tree
        start = time.time()
        self.euclidean_clustering_ec(ec_cloud, ec_tree)
        clustering_ec_time = time.time() - start
        self.get_logger().info(f"PCL binding ec took {clustering_ec_time:.5f} to find {len(self.clusters_ec)} clusters.")

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
        pcl2_msg = pcl_to_ros(cluster_colour_cloud, timestamp) # convert the pcl to a ROS PCL2 message
        self._cloud_cluster_publisher.publish(pcl2_msg)

        # fit bounding boxes to the clustered pointclouds
        detected_objects = self.fit_bounding_boxes() 

        # track objects over time
        tracked_detected_objects = self.tracker.update(detected_objects)
        # add labels
        print(f"Number of tracked objects: {len(tracked_detected_objects.detected_objects)}")
        # add a marker for every tracked object
        for d_o in tracked_detected_objects.detected_objects:
            object_marker = Marker()
            object_marker.id = d_o.object_id
            object_marker.ns = 'object_id'
            object_marker.header.frame_id = 'velodyne'
            object_marker.type = Marker.TEXT_VIEW_FACING
            object_marker.action = object_marker.ADD
            object_marker.pose.position.x = d_o.pose.position.x
            object_marker.pose.position.y = d_o.pose.position.y + 0.5
            object_marker.pose.position.z = d_o.pose.position.z
            object_marker.color.a = 1.
            object_marker.color.r = 1.
            object_marker.color.g = 1.
            object_marker.color.b = 1.
            object_marker.scale.x = 1.
            object_marker.scale.y = 0.8
            object_marker.scale.z = 0.5
            object_marker.text = f"{d_o.object_id}"
            self.bounding_box_markers.markers.append(object_marker)
        # remove the old markers from the previous frame
        try:
            tracked_ids = [obj.object_id for obj in tracked_detected_objects.detected_objects]
            marker_ids = [marker.id for marker in self.bounding_box_markers.markers]
            print(f"Tracked IDs: {tracked_ids}")
            print(f"Marker IDs: {marker_ids}")
            for old_id in range(max(tracked_ids)):
                if old_id not in marker_ids:
                    del_marker = Marker()
                    del_marker.ns = 'object_id'
                    del_marker.header.frame_id = 'velodyne'
                    del_marker.id = old_id
                    del_marker.action = Marker.DELETE
                    self.bounding_box_markers.markers.append(del_marker)
        except:
            pass
        
        print(f"Number of bounding box markers: {len(self.bounding_box_markers.markers)}")
        self._bounding_boxes_publisher.publish(self.bounding_box_markers)
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

    def fit_bounding_boxes(self):
        """
        Fit bounding boxes to clusters identified.
        
        Tutorial at PCL docs helps with make_MomentOfInertiaEstimation aspect
        https://pcl.readthedocs.io/projects/tutorials/en/master/moment_of_inertia.html#moment-of-inertia
        """
        self.bounding_box_markers = MarkerArray() # list of markers for visualisations of boxes
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
            # convert rotational matrix to quaternion for use in ROS pose
            quat = mat2quat(rotational_matrix_OBB)

            # create marker box for visualisation
            marker = Marker()
            marker.id = cluster_idx + 1000
            marker.ns = 'bounding_boxes'
            marker.header.frame_id = 'velodyne'
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.color.a = 0.5
            marker.color.r = self.rgb_list[cluster_idx][0]/255.
            marker.color.g = self.rgb_list[cluster_idx][1]/255.
            marker.color.b = self.rgb_list[cluster_idx][2]/255.

            #### axis-aligned version
            # marker.scale.x = float(abs(max_point_AABB[0,0]-min_point_AABB[0,0]))
            # marker.scale.y = float(abs(max_point_AABB[0,1]-min_point_AABB[0,1]))
            # marker.scale.z = float(abs(max_point_AABB[0,2]-min_point_AABB[0,2]))
            # marker.pose.orientation.w = 1.
            # marker.pose.position.x = float(max_point_AABB[0,0]+min_point_AABB[0,0])/2
            # marker.pose.position.y = float(max_point_AABB[0,1]+min_point_AABB[0,1])/2
            # marker.pose.position.z = float(max_point_AABB[0,2]+min_point_AABB[0,2])/2
            #### oriented version
            # size -> 2 times the max_point from centre
            marker.scale.x = 2*float(max_point_OBB[0,0])
            marker.scale.y = 2*float(max_point_OBB[0,1])
            marker.scale.z = 2*float(max_point_OBB[0,2])
            # marker.pose.orientation.w = 1.
            mag = sqrt(quat[0]**2 + quat[3]**2)
            marker.pose.orientation.w = float(quat[0]/mag)
            marker.pose.orientation.x = 0.#float(quat[1])
            marker.pose.orientation.y = 0.#float(quat[2]/mag)
            marker.pose.orientation.z = float(quat[2]/mag)#float(quat[3])
            marker.pose.position.x = float(position_OBB[0,0])
            marker.pose.position.y = float(position_OBB[0,1])
            marker.pose.position.z = float(position_OBB[0,2])

            self.bounding_box_markers.markers.append(marker)

            # create detected object
            detected_object = DetectedObject()
            detected_object.object_id = cluster_idx # dummy value until we track the objects
            detected_object.frame_id = 'velodyne'
            # pose -> assume of center point
            detected_object.pose.orientation.w = 1.
            detected_object.pose.position.x = float(position_OBB[0,0])
            detected_object.pose.position.y = float(position_OBB[0,1])
            detected_object.pose.position.z = float(position_OBB[0,2])
            # dimensions -> assuming want the length of vertex
            detected_object.dimensions.x = 2*float(max_point_OBB[0,0])
            detected_object.dimensions.y = 2*float(max_point_OBB[0,1])
            detected_object.dimensions.z = 2*float(max_point_OBB[0,2])
            # object and signal class -> unknown for now
            detected_object.object_class = detected_object.CLASS_UNKNOWN
            detected_object.signal_state = detected_object.SIGNAL_UNKNOWN

            objects.detected_objects.append(detected_object)

        bounding_ids = [marker.id for marker in self.bounding_box_markers.markers]
        for old_id in range(1000, 1000+40):
            if old_id not in bounding_ids:
                del_marker = Marker()
                del_marker.ns = 'bounding_boxes'
                del_marker.header.frame_id = 'velodyne'
                del_marker.id = old_id
                del_marker.action = Marker.DELETE
                self.bounding_box_markers.markers.append(del_marker)

        print(f"{len(self.bounding_box_markers.markers)} bounding boxes found")
        

        return objects



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