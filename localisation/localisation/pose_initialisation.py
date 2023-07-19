"""
Takes input from /rough_pose_guess, /pointcloud_map, /lidar_points to perform local
registration of the pointclouds to find the Transform between the two frames /map and 
/lidar_points. This refines the initial pose estimate using local registration to 
get an accurate /initial_pose_transform and publishes the Transform
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import open3d as o3d # Open3d for ICP implementation
import numpy as np
import math

class PoseInitialisation(Node):
    def __init__(self):
        super().__init__('pose_initialisation')
        
        # Subscriber to /rough_pose_guess
        self.rough_pose_guess_sub = self.create_subscription(PoseStamped,
            '/rough_pose_guess', self.rough_pose_callback, 10)
        
        # Subscriber to /pointcloud_map
        self.pointcloud_map_sub = self.create_subscription(PointCloud2,
            '/pointcloud_map', self.set_map, 10)
            
        # Subcriber to /lidar_points 
        self.lidar_points_sub = self.create_subscription(PointCloud2,
            '/lidar_points', self.set_lidar_points, 10)
            
        # Publisher for transform from map frame to lidar frame
        self.transform_pub = self.create_publisher(TransformStamped, '/initial_pose_transform', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds
        
        # Listened inputs
        self.map = None
        self.rough_pose_guess = None
        self.lidar_points = None
        
        # Results to be published
        self.correction_transform = None
        
        # Run ICP until below threshold error or max iterations reached
        self.threshold = 0.02 # Local registration threshold for RMSE of pose estimate
        self.max_iteration = 1000 # Maximum number of iterations of ICP performed
        
    def rough_pose_callback(self, pose_msg: PoseStamped):
        """
        After receiving rough pose guess, if the lidar points and reference map have been received, 
        find the current pose by the transform from the reference map to the frame of the lidar points 
        measured using Iterative-Closest-Point for local registration.
        """
        self.get_logger().info('Received rough pose guess')
        self.rough_pose_guess = pose_msg
        # Run local registration on point clouds if both have been received
        # Assumes remaining stationary until the initial pose is determined
        if self.map is not None and self.lidar_points is not None:
            self.get_logger().info('Received inputs, finding initial pose')
            
            # Convert ROS2 messages into open3D data types
            # Converting ROS2 PointCloud2 to Open3D PointCloud
            source = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pc2.read_points(self.map)))
            target = o3d.geometry.PointCloud(
                o3d.utility.Vector3dVector(pc2.read_points(self.lidar_points)))
            # Converting rough pose guess ROS2 PoseStamped 
            # to an affine transformation matrix
            pose = self.inital_pose_guess.pose
            
            # Alternatively: o3d.geometry.get_rotation_matrix_from_quaternion exists
            rot_matrix = quaternion_to_rotation_matrix(pose.orientation)
            
            transform_guess = np.eye(4)
            transform_guess[0:3, 0:3] = rot_matrix
            transform_guess[0:3, 3] = np.asarray([pose.position.x,pose.position.y,pose.position.z]).T
            # Optional: Choose between Point to Point or Point to Plane ICP method (Point to Plane can perform better if the environment has flat surfaces)
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, self.threshold, transform_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = self.max_iteration))
            result = reg_p2p.transformation # This is a 4x4 transformation matrix
            
            rotation = rotation_matrix_to_quaternion(result[0:3, 0:3])
            translation = Vector3(result[0, 3], result[1, 3], result[2, 3])
            self.correction_transform = Transform(translation, rotation)
            
    def set_map(self, pointcloud_msg: PointCloud2):
        if self.map is None:
            self.get_logger().info('Received map point cloud')
        self.map = pointcloud_msg
        
    def set_lidar_points(self, pointcloud_msg: PointCloud2):
        if self.lidar_points is None:
            self.get_logger().info('Received lidar point cloud')
        self.lidar_points = pointcloud_msg
        
    def timer_callback(self):
        if self.correction_transform is not None:
            msg = TransformStamped()
            msg.header.frame_id = 'map_transform_lidar' # This is the approximate transform that places the map onto the lidar pointcloud
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.transform = self.correction_transform
            self.transform_pub.publish(msg)
            
    def quaternion_to_rotation_matrix(quaternion):
        """
        Input: ROS2 Quaternion
        Output: 3x3 Rotation matrix as a numpy array
        Using a quaternion, q, to rotate p to p' is p'=q*p*q^-1, a rotation matrix is of the form p' = Rp
        # Quarternion values as coefficients in (w*r,x*i,y*j,z*k) for (w,x,y,z)
        Assumes unit quaternions, maybe
        """
        x,y,z,w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        s = (x**2+y**2+z**2+w**2) **-2 # Should be 1 for unit quaternions
        # TODO: Check that this was transcribed correctly
        # Alternatively: o3d.geometry.get_rotation_matrix_from_quaternion exists
        rot_matrix = np.asarry([[1 - 2*s*(y**2 + z**2), 2*s*(x*y - z*w), 2*s*(y*z + y*w)],
                                     [2*s*(x*y + z*w), 1-2*s*(x**2+z**2), 2*s*(y*z - x*w)],
                                     [2*s*(x*z - y*w), 2*s*(y*z + x*w), 1 - 2*s*(x**2 + y**2)]])
        return rot_matrix
        
    def rotation_matrix_to_quarternion(rot_matrix):
        """Input: a 3x3 rotation matrix as a numpy array
           Output: a ROS2 Quaternion representing the same rotation
           # Using the identity of trace + 1 = 4w^2 and the unit quaternion definition
           """
        trace = rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]
        r = math.sqrt(trace)
        s = 1/(2*r)
        w = 0.5*r 
        x = (rot_matrix[2, 1] - rot_matrix[1, 2])*s
        y = (rot_matrix[0, 2] - rot_matrix[2, 0])*s
        z = (rot_matrix[1, 0] - rot_matrix[0, 1])*s
        return Quaternion(x=x,y=y,z=z,w=w)
    
def main(args=None):
    rclpy.init(args=args)
    poseInitialise = PoseInitialisation()
    rclpy.spin(poseInitialise)
    poseInitialise.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
