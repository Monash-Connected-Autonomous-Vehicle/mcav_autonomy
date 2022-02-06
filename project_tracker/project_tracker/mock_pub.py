import rclpy
from rclpy.node import Node

import numpy as np
import glob 

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

class PointCloudToPCL2(Node):

    def __init__(self):
        super(PointCloudToPCL2, self).__init__('point_cloud_to_pcl2')
        self._publisher = self.create_publisher(PCL2, 'pcl2conversion', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_pcl2)

        self.velodyne_file_paths = glob.glob('install/project_tracker/lib/project_tracker/data/velodyne_points/*.bin')

    def publish_pcl2(self):
        """Callback to publish"""
        
        if self.velodyne_file_paths:
            msg = self.convert_bin_to_PCL2(self.velodyne_file_paths.pop())
            self._publisher.publish(msg)
        else:
            self.get_logger().info("no more velodyne points to publish...")

    def convert_bin_to_PCL2(self, velodyne_file_path):
        """Method to convert Lidar data in binary format to PCL2 message"""
        
        cloud = np.fromfile(velodyne_file_path, np.float32)
        cloud = cloud.reshape((-1, 4))
        # x, y, z, r = cloud[::4], cloud[1::4], cloud[2::4], cloud[3::4]

        header = Header()
        header.frame_id = 'velodyne'
        header.stamp = self.get_clock().now().to_msg()
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        pcl2_msg = point_cloud2.create_cloud(header, fields, cloud)

        self.get_logger().info(f"Publishing, first point: {cloud[0:5]}")

        return pcl2_msg


def main(args=None):
    rclpy.init(args=args)

    point_cloud_to_pcl2 = PointCloudToPCL2()

    try:
        rclpy.spin(point_cloud_to_pcl2)
    except KeyboardInterrupt:
        point_cloud_to_pcl2.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    point_cloud_to_pcl2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
