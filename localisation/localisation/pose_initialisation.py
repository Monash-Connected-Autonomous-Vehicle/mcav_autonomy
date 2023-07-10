"""
Takes input from /initial_pose_guess, /pointcloud_map, /lidar_points to perform local
registration of the pointclouds to find the Transform between the two frames /map and 
/initial_pose. This refines the initial pose estimate using local registration to 
get an accurate /initial_pose_transform and publishes the Transform between the /pointcloud_map to the lidar corrected pose
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import open3d as o3d # Open3d for ICP implementation

class PoseInitialisation(Node):
    def __init__(self):
        super().__init__('pose_initialisation')
        
        # Subscriber to /initial_pose_guess
        self.initial_pose_guess_sub = self.create_subscription(PoseStamped,
            '/initial_pose_guess', self.initial_pose_callback, 10)
        
        # Subscriber to /pointcloud_map
        self.pointcloud_map_sub = self.create_subscription(PointCloud2,
            '/pointcloud_map', self.set_map, 10)
            
        # Subcriber to /lidar_points 
        self.lidar_points_sub = self.create_subscription(PointCloud2,
            '/lidar_points', self.set_lidar_points, 10)
            
        # Publisher # TODO: determine out topic name to publish to
        self.transform_pub = self.create_publisher(TransformStamped, '/initial_pose_transform', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds
        
        # Listened inputs
        self.map = None
        self.initial_pose_guess = None
        self.lidar_points = None
        
        # Results to be published
        self.correction_transform = None
        
        # Run ICP until below threshold error or max iterations reached
        self.threshold = 0.02 # Local registration threshold for RMSE of pose estimate
        self.max_iteration = 1000 # Maximum number of iterations of ICP performed
        
    def initial_pose_callback(self, pose_msg: PoseStamped):
        """
        After receiving initial pose guess, if the lidar points and reference map have been received, 
        find the current pose by the transform from the reference map to the frame of the lidar points 
        measured using Iterative-Closest-Point for local registration.
        """
        if self.initial_pose_guess is None:
            self.get_logger().info('Received initial pose guess')
        self.initial_pose_guess = pose_msg
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
            # Converting initial pose guess ROS2 PoseStamped 
            # to an affine transformation matrix
            pose = self.inital_pose_guess.pose
            # Quarternion values as coefficients in (w*r,x*i,y*j,z*k) for (w,x,y,z)
            w,x,y,z = pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
            # s = np.abs(Pose.orientation) **-2 # Should be 1 for unit quaternions
            # TODO: Check that this was transcribed correctly
            # Alternatively: o3d.geometry.get_rotation_matrix_from_quaternion exists
            transform_guess = np.asarry([[1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(y*z + y*w), pose.point.x],
                                         [2*(x*y + z*w), 1-2*(x**2+z**2), 2*(y*z - x*w), pose.point.y],
                                         [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2), pose.point.z],[0,0,0,1]])
            # Optional: Choose between Point to Point or Point to Plane ICP method (Point to Plane can perform better if the environment has flat surfaces)
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source, target, self.threshold, transform_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = self.max_iteration))
            result = reg_p2p.transformation # This is a 4x4 transformation matrix
            
            # Convert to a Transform (quaternion and translation vector) to publish 
            # TODO: Decide on a stable method to find a quaternion to represent the rotation matrix
            # Using the identity of trace + 1 = 4w^2
            trace = result[0, 0] + result[1, 1] + result[2, 2]
            r = trace**0.5
            s = 1/(2*r)
            w = 0.5*r 
            x = (result[2, 1] - result[1, 2])*s
            y = (result[0, 2] - result[2, 0])*s
            z = (result[1, 0] - result[0, 1])*s
            rotation = Quaternion(x,y,z,w)
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
            # TODO: find frame id of lidar
            msg.header.frame_id = 'lidar' # This is a temp name
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.transform = self.correction_transform
            self.transform_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    poseInitialise = PoseInitialisation()
    rclpy.spin(poseInitialise)
    poseInitialise.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
