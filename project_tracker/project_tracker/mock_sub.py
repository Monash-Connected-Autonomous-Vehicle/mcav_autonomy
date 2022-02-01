import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class PCL2Subscriber(Node):

    def __init__(self):
        super(PCL2Subscriber, self).__init__('pcl2_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic1',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard {msg.data}')


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