"""
From a reference map and lidar pointcloud, use global registration to create an rough pose guess
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Transform, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import open3d as o3d # Open3d for ICP and global registration implementation
import numpy as np
import math

class RoughPoseGuess(Node):
    def __init__(self):
        super().__init__('rough_pose_guess')
        # Subscriber to /pointcloud_map
        self.pointcloud_map_sub = self.create_subscription(PointCloud2,
            '/pointcloud_map', self.set_map, 10)
            
        # Subcriber to /lidar_points 
        self.lidar_points_sub = self.create_subscription(PointCloud2,
            '/lidar_points', self.set_lidar_points, 10)
            
        # Publisher for rough pose guess # Or a transform?
        self.transform_pub = self.create_publisher(TransformStamped, '/rough_pose_guess', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds
        
        #Global registration parameters
        self.voxel_size = 0.2 # Decrease to increase fitness and reduce rmse, results in more correspondences found.
        
        # Listened inputs
        self.map = None
        self.lidar_points = None
        
        # Stored output
        self.rough_pose_guess = None
        
    def set_map(self, pointcloud_msg: PointCloud2):
        if self.map is None:
            self.get_logger().info('Received map point cloud')
        self.map = pointcloud_msg
        
    def set_lidar_points(self, pointcloud_msg: PointCloud2):
        if self.lidar_points is None:
            self.get_logger().info('Received lidar point cloud')
        self.lidar_points = pointcloud_msg
        
    def timer_callback(self):
        """Run global registration on the map and lidar pointclouds to find a rough alignment transformation"""
        if self.map is not None and self.lidar_points is not None:
            # self.get_logger().info('Found inputs')
            
            # Convert ROS2 messages into open3D data types
            # Converting ROS2 PointCloud2 to Open3D PointCloud

            source = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pc2.read_points_numpy(self.map)))
            target = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pc2.read_points_numpy(self.lidar_points)))
            
            source_down, source_fpfh = preprocess_point_cloud(source, self.voxel_size)
            target_down, target_fpfh = preprocess_point_cloud(target,self.voxel_size)
            result_ransac = execute_fast_global_registration(source_down, target_down,
                                                        source_fpfh, target_fpfh,
                                                        self.voxel_size)
            # After this, either ICP for local registration with the initial transform produced
            # Or instead the fast global registration can be used
            # What is the error required?
            trans_matrix = result_ransac.transformation
            self.get_logger().info(str(result_ransac))
            # self.get_logger().info(f"Rotation: {trans_matrix[0:3, 0:3]}")
            # self.get_logger().info(f"Trace: {sum(np.diag(trans_matrix[0:3,0:3]))}")
            rotation = rotation_matrix_to_quarternion(trans_matrix[0:3, 0:3])

            translation = Vector3(x=trans_matrix[0,3],y=trans_matrix[1,3],z=trans_matrix[2,3])
            self.rough_pose_guess = Transform(translation=translation, rotation=rotation)
            # Publish message
            msg = TransformStamped()
            msg.header.frame_id = 'map_transform_lidar' # This is the rough transform that places the map onto the lidar pointcloud
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.transform = self.rough_pose_guess
            self.transform_pub.publish(msg)
            
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh
            
def execute_global_registration(source_down, target_down, source_fpfh,
                            target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result
    
def rotation_matrix_to_quarternion(rot_matrix):
    """Input: a 3x3 rotation matrix as a numpy array
       Output: a ROS2 Quaternion representing the same rotation
       # Using the identity of trace + 1 = 4w^2 and the unit quaternion definition
       """
    trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]
    r = math.sqrt(1+trace)
    s = 1/(2*r)
    w = 0.5*r 
    x = (rot_matrix[2, 1] - rot_matrix[1, 2])*s
    y = (rot_matrix[0, 2] - rot_matrix[2, 0])*s
    z = (rot_matrix[1, 0] - rot_matrix[0, 1])*s
    return Quaternion(x=x,y=y,z=z,w=w)
   
def main(args=None):
    rclpy.init(args=args)
    poseGuess = RoughPoseGuess()
    rclpy.spin(poseGuess)
    poseGuess.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
