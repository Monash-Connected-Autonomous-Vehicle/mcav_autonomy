import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 as PCL2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

class PCL2Subscriber(Node):

    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            PCL2,
            'pcl2conversion',
            self._callback,
            10
        )

    def _callback(self, msg):
        """Subscriber callback. Receives PCL2 message and converts it to points"""
        cloud_generator = point_cloud2.read_points(msg)
        # TODO figure out what to do with generator properly
        cloud = list(cloud_generator)
        self.get_logger().info(f"Received, first point: {cloud[0]}")
        return cloud
        


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