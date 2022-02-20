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
        # self._cloud_cluster_publisher = self.create_publisher(PCL2, '/cloud_clusters', )
        # self._detected_objects_publisher = self.create_publisher(DetectedObjectArray, 10)

        # parameters for Euclidean Clustering
        self.min_cluster_size = 4

        self.get_logger().set_level(logging.DEBUG)

    def _callback(self, msg):
        """Subscriber callback. Receives PCL2 message and converts it to points"""
        start = time.time()
        cloud_generator = point_cloud2.read_points(msg)
        # convert cloud to numpy array
        cloud_np = np.fromiter(chain.from_iterable(cloud_generator), np.float32)
        cloud_np = cloud_np.reshape((-1, 4))
        # discard reflectance
        self.cloud = cloud_np[:, :3]
        self.cloud = self.cloud[:2000]
        self.no_samples, self.no_axes = self.cloud.shape
        cloud_time = time.time() - start
        self.get_logger().info(f"Took {cloud_time:.5f} to receive pcl2 message: {self.no_samples, self.no_axes}\n{self.cloud[0]}\n")

        # create kd-tree
        tree = KDTree(self.cloud, leaf_size=2)
        start = time.time()
        self.euclidean_clustering(tree)
        clustering_time = time.time() - start
        self.get_logger().info(f"Took {clustering_time:.5f} to find {len(self.clusters)} clusters.")
        # for i, cluster in enumerate(self.clusters):
            # self.get_logger().info(f"Cluster {i+1}: {len(cluster)} points")

    def euclidean_clustering(self, tree):
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
                self.find_clusters(point, index, base_cluster, tree)

                try:
                    if len(base_cluster) > self.min_cluster_size:
                        self.clusters.append(base_cluster)
                except TypeError as e:
                    print(e)
                    self.get_logger().debug("base_cluster not a set.")
        
        return self.clusters

    def find_clusters(self, point, ind, base_cluster, tree):
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
            nearby_indices = tree.query_radius(point_exp, r=0.3,)# return_distance=True)
            # self.get_logger().debug(f"Point: {ind} has {len(nearby_indices[0])}")
            for nearby_ind in nearby_indices[0]:
                nearby_ind_int = int(nearby_ind)
                if nearby_ind_int != ind:
                    if not self.processed[nearby_ind_int]:
                        # self.get_logger().debug(f"Single indice {nearby_ind}")
                        self.find_clusters(self.cloud[nearby_ind_int], nearby_ind_int, base_cluster, tree)
        


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