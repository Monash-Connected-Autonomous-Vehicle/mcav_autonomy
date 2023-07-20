import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistStampedPublisher(Node):
    def __init__(self):
        super().__init__('twist_cmd')
        self.publisher_ = self.create_publisher(TwistStamped, 'twist_cmd', 10)
        self.timer_ = self.create_timer(1.0, self.publish_twist_stamped)
        self.get_logger().info('TwistStamped publisher node has been initialized')

    def publish_twist_stamped(self):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()

        twist_stamped_msg.twist.linear.x = 0.004
        twist_stamped_msg.twist.linear.y = 0.0
        twist_stamped_msg.twist.linear.z = 0.0

        twist_stamped_msg.twist.angular.x = 0.0
        twist_stamped_msg.twist.angular.y = 0.0
        twist_stamped_msg.twist.angular.z = 0.02

        self.publisher_.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    twist_stamped_publisher = TwistStampedPublisher()
    rclpy.spin(twist_stamped_publisher)
    twist_stamped_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
