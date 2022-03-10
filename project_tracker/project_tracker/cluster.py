import rclpy
from rclpy.node import Node

import numpy as np
import logging
import time
from itertools import chain

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from project_tracker.utils import numpy_2_PCL2

from sklearn.neighbors import KDTree
import pcl

class PCL2Subscriber(Node):

    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            PCL2,
            '/velodyne_points',
            self._callback,
            10
        )

        # TODO create cloud cluster publisher via creating a custom msg
        self._cloud_cluster_publisher = self.create_publisher(PCL2, 'clustered_pointclouds', 10)

        # parameters for Euclidean Clustering
        self.min_cluster_size = 10
        self.max_search_radius = 0.1

        self.get_logger().set_level(logging.DEBUG)

    def _callback(self, msg):
        """Subscriber callback. Receives PCL2 message and converts it to points"""
        start = time.time()
        cloud_generator = point_cloud2.read_points(msg)

        # convert cloud to numpy array
        cloud_np = np.fromiter(chain.from_iterable(cloud_generator), np.float32)
        cloud_np = cloud_np.reshape((-1, 4))
        
        # discard reflectance
        self.full_cloud = cloud_np
        self.cloud = cloud_np[:, :3] 
        
        self.no_samples, self.no_axes = self.cloud.shape
        cloud_time = time.time() - start
        self.get_logger().info(f"Took {cloud_time:.5f} to receive pcl2 message: {self.no_samples, self.no_axes}\n{self.cloud[0]}\n")

        ## python3-pcl binding euclidean clustering
        ec_cloud = pcl.PointCloud()
        ec_cloud.from_array(self.cloud)
        ec_tree = ec_cloud.make_kdtree()
        start = time.time()
        self.euclidean_clustering_ec(ec_cloud, ec_tree)
        clustering_ec_time = time.time() - start
        self.get_logger().info(f"PCL binding ec took {clustering_ec_time:.5f} to find {len(self.clusters_ec)} clusters.")

        # convert and publish clustered pointcloud
        # stupid way of doing this
        cluster_indices = set()
        for single_cluster_indices in self.clusters_ec:
            cluster_indices.update(single_cluster_indices)
        clustered_np_array = self.full_cloud[list(cluster_indices)]

        # publish message
        timestamp = self.get_clock().now().to_msg()
        pcl2_msg = numpy_2_PCL2(clustered_np_array, timestamp)
        self._cloud_cluster_publisher.publish(pcl2_msg)

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
        ec.set_ClusterTolerance(self.max_search_radius)
        ec.set_MinClusterSize(self.min_cluster_size + 1)
        ec.set_MaxClusterSize(100)
        ec.set_SearchMethod(ec_tree)
        # perform euclidean clustering and return indices
        self.clusters_ec = ec.Extract()
        return

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