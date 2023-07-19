import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, Transform, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import std_msgs
import open3d as o3d # Open3d for ICP implementation
import numpy as np

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.map_pub = self.create_publisher(PointCloud2, 
            '/pointcloud_map', 10)
        self.lidar_pub = self.create_publisher(PointCloud2,
            '/lidar_points', 10)
            
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        demo_icp_pcds = o3d.data.DemoICPPointClouds()
        
        source = pc2.create_cloud_xyz32(header = std_msgs.msg.Header(),
            points = np.asarray(o3d.io.read_point_cloud(demo_icp_pcds.paths[0]).points))
        array = pc2.read_points(source)
        self.get_logger().info(f"Size: {array.size}")
        self.get_logger().info(f"Point 0: {array[0]}")
        target = pc2.create_cloud_xyz32(header = std_msgs.msg.Header(),
            points = np.asarray(o3d.io.read_point_cloud(demo_icp_pcds.paths[1]).points))
        self.map_pub.publish(source)
        self.lidar_pub.publish(target)
        
        
def main(args=None):
    rclpy.init(args=args)
    testPublisher = TestPublisher()
    rclpy.spin(testPublisher)
    testPublisher.destroyNode()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
