import rclpy
from rclpy.node import Node

import logging
import time

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from project_tracker.utils import numpy_2_PCL2, PCL2_2_numpy, create_colour_list, pcl_to_ros

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

        # parameters for Euclidean Clustering
        self.min_cluster_size = 10
        self.max_cluster_size = 1000
        self.max_search_radius = 0.5 # range from 0.5 -> 0.7 seems suitable. Test more when have more data

        # colour list for publishing different clusters
        self.colour_list = create_colour_list()

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
        ec_cloud = pcl.PointCloud()
        ec_cloud.from_array(self.cloud)
        ec_tree = ec_cloud.make_kdtree()
        start = time.time()
        self.euclidean_clustering_ec(ec_cloud, ec_tree)
        clustering_ec_time = time.time() - start
        self.get_logger().info(f"PCL binding ec took {clustering_ec_time:.5f} to find {len(self.clusters_ec)} clusters.")

        ## PUBLISHING
        # COLOURED VERSION
        # colouring the clouds
        colour_cluster_point_list = []
        for j, indices in enumerate(self.clusters_ec):
            for indice in indices:
                colour_cluster_point_list.append([
                    self.cloud[indice][0],
                    self.cloud[indice][1],
                    self.cloud[indice][2],
                    self.colour_list[j]
                ])

        cluster_colour_cloud = pcl.PointCloud_PointXYZRGB()
        cluster_colour_cloud.from_list(colour_cluster_point_list)
        timestamp = self.get_clock().now().to_msg()
        pcl2_msg = pcl_to_ros(cluster_colour_cloud, timestamp)
        self._cloud_cluster_publisher.publish(pcl2_msg)

        # COMBINING ALL INTO SINGLE POINTCLOUD
        # cluster_indices = set()
        # for single_cluster_indices in self.clusters_ec:
        #     cluster_indices.update(single_cluster_indices)
        # clustered_np_array = self.full_cloud[list(cluster_indices)]

        # # publish message
        # timestamp = self.get_clock().now().to_msg()
        # pcl2_msg = numpy_2_PCL2(clustered_np_array, timestamp, reflectance=False)
        # self._cloud_cluster_publisher.publish(pcl2_msg)

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
        ec.set_MaxClusterSize(self.max_cluster_size)
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