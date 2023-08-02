import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud2
class Converter(Node):
    """Interpret streetdrone stack topics for gazebo simulation:
        -Unstamp Twist msgs"""
    def __init__(self):
        super().__init__('converter')
        self.twist_sub = self.create_subscription(TwistStamped, "/twist_cmd", self.twist_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "/simulated_vehicle/cmd_vel", 10)
        self.scanner_sub = self.create_subscription(LaserScan, "/demo/laser/out", 10)
        self.pc2_pub = self.create_publisher(PointCloud2, "/velodyne_points", 10)
    
    def twist_callback(self, msg):
        self.twist_pub.publish(msg.twist)

    def scan_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
