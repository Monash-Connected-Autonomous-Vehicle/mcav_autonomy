"""Since the ros2 bag command replays data with old timestamps, 
(See https://github.com/ros2/rosbag2/issues/1056)
this node listens to the provided topic and republishes data 
with updated timestamps."""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

class LidarStampUpdater(Node):

    def __init__(self):
        super().__init__('lidar_stamp_updater')

        self.lidar_pub = self.create_publisher(PointCloud2, 'velodyne_points', 10)
        self.lidar_sub = self.create_subscription(PointCloud2,
            'lidar_points_outdated', self.lidar_callback, 10)
        self.lidar_sub  # prevent unused variable warning

        self.get_logger().info('StampUpdater initialised.')

    def lidar_callback(self, msg: PointCloud2):
        """ Updates timestamp """
        msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    stamp_updater = LidarStampUpdater()
    rclpy.spin(stamp_updater)
    stamp_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
