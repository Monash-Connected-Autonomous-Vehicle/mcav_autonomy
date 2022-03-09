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
# from mcav_interfaces import DetectedObjectArray

from sklearn.neighbors import KDTree
import pcl

class PCL2Subscriber(Node):

    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            PCL2,
            'pcl2conversion',
            self._callback,
            10
        )

        # TODO create cloud cluster publisher via creating a custom msg
        self._cloud_cluster_publisher = self.create_publisher(PCL2, 'clustered_pointclouds', 10)
        # self._detected_objects_publisher = self.create_publisher(DetectedObjectArray, 10)

        # parameters for Euclidean Clustering
        self.min_cluster_size = 4
        self.max_search_radius = 0.3

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
        self.cloud = self.cloud[:2000]
        self.no_samples, self.no_axes = self.cloud.shape
        cloud_time = time.time() - start
        self.get_logger().info(f"Took {cloud_time:.5f} to receive pcl2 message: {self.no_samples, self.no_axes}\n{self.cloud[0]}\n")

        ### 
        # ! Examining cluster detection time of several methods
        # ! - sklearn KDTree
        # ! - using octree search_radius
        # ! - using builtin euclidean clustering method from PCL
        ###

        # ## sklearn
        # # create kd-tree
        # tree = KDTree(self.cloud, leaf_size=2)
        # start = time.time()
        # self.euclidean_clustering_sklearn(tree)
        # clustering_time = time.time() - start
        # self.get_logger().info(f"Sklearn KDTree took {clustering_time:.5f} to find {len(self.clusters)} clusters.")

        # ## python3-pcl binding octree
        # # create octree
        # octree_cloud = pcl.PointCloud()
        # octree_cloud.from_array(self.cloud)
        # tree = octree_cloud.make_octreeSearch(0.05) # 0.05 is resolution -> length of smallest voxels at lowest octree level
        # tree.add_points_from_input_cloud()
        # start = time.time()
        # self.euclidean_clustering_octree(tree)
        # clustering_octree_time = time.time() - start
        # self.get_logger().info(f"PCL binding octree took {clustering_octree_time:.5f} to find {len(self.clusters_octree)} clusters.")
        
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
        header = Header()
        header.frame_id = 'clustered_pointclouds'
        header.stamp = self.get_clock().now().to_msg()
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        pcl2_msg = point_cloud2.create_cloud(header, fields, clustered_np_array)
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
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(ec_tree)
        # perform euclidean clustering and return indices
        self.clusters_ec = ec.Extract()
        return self.clusters_ec

    
    def euclidean_clustering_octree(self, tree):
        """
        Perform euclidean clustering with a given octree with the python-pcl binding.

        Parameters
        ----------
        tree : octree
            octree setup of the point cloud to search
        """
        self.clusters_octree = []
        self.processed = [False] * self.no_samples # track whether point in tree already processed
        for index, point in enumerate(self.cloud):
            if not self.processed[index]:
                # create set here to not reset it at every recursive call to find_clusters
                base_cluster = set()
                self.find_clusters_octree(point, index, base_cluster, tree)
                try:
                    if len(base_cluster) > self.min_cluster_size:
                        self.clusters_octree.append(base_cluster)
                except TypeError as e:
                    print(e)
                    self.get_logger().debug("base_cluster not a set.")
        
        return self.clusters_octree

    def find_clusters_octree(self, point, ind, base_cluster, tree):
        """
        Find clusters for a given point recursively.
        
        Parameters
        ----------
        point : np.array with shape (1, 3)
            Represents a single datapoint from a point cloud.
        ind : int
            Index of point within the point cloud. Needed for recursion
        base_cluster : set
            Set to represent all the points in a cluster
        tree : octree
            octree setup of the point cloud to search
        """
        if not self.processed[ind]:
            self.processed[ind] = True # point now been processed
            base_cluster.add(ind)
            # tuple format for searching
            search_point_tup = (point[0], point[1], point[2])
            [nearby_indices, _] = tree.radius_search(search_point_tup, self.max_search_radius)
            for nearby_ind in nearby_indices:
                nearby_ind_int = int(nearby_ind)
                if nearby_ind_int != ind:
                    if not self.processed[nearby_ind_int]:
                        self.find_clusters_octree(self.cloud[nearby_ind_int], nearby_ind_int, base_cluster, tree)
        

    def euclidean_clustering_sklearn(self, tree):
        """
        Perform euclidean clustering with a given kd-tree.

        Parameters
        ----------
        tree : KDTree
            kd-tree setup of the point cloud to search
        """
        self.clusters = []
        self.processed = [False] * self.no_samples # track whether point in tree already processed
        for index, point in enumerate(self.cloud):
            if not self.processed[index]:
                # create set here to not reset it at every recursive call to find_clusters
                base_cluster = set()
                self.find_clusters_sklearn(point, index, base_cluster, tree)

                try:
                    if len(base_cluster) > self.min_cluster_size:
                        self.clusters.append(base_cluster)
                except TypeError as e:
                    print(e)
                    self.get_logger().debug("base_cluster not a set.")
        
        return self.clusters

    def find_clusters_sklearn(self, point, ind, base_cluster, tree):
        """
        Find clusters for a given point recursively.
        
        Parameters
        ----------
        point : np.array with shape (1, 3)
            Represents a single datapoint from a point cloud.
        ind : int
            Index of point within the point cloud. Needed for recursion
        base_cluster : set
            Set to represent all the points in a cluster
        tree : KDTree
            kd-tree setup of the point cloud to search
        """
        if not self.processed[ind]:
            self.processed[ind] = True # point now been processed
            base_cluster.add(ind)
            point_exp = np.expand_dims(point, 0) # required shape for query_radius
            nearby_indices = tree.query_radius(point_exp, r=self.max_search_radius)# return_distance=True)
            for nearby_ind in nearby_indices[0]:
                nearby_ind_int = int(nearby_ind)
                if nearby_ind_int != ind:
                    if not self.processed[nearby_ind_int]:
                        self.find_clusters_sklearn(self.cloud[nearby_ind_int], nearby_ind_int, base_cluster, tree)
        


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