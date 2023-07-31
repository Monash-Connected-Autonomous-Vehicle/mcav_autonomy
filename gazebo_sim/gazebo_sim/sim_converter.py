import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist

class Converter(Node):
    """Interpret streetdrone stack topics for gazebo simulation:
        -Unstamp Twist msgs"""
    def __init__(self):
        super().__init__('converter')
        self.twist_sub = self.create_subscription(TwistStamped, "/twist_cmd", self.twist_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "/simulated_vehicle/cmd_vel", 10)
    
    def twist_callback(self, msg):
        self.twist_pub.publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
