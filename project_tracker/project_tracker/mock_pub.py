import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class PointCloudToPCL2(Node):

    def __init__(self):
        super(PointCloudToPCL2, self).__init__('point_cloud_to_pcl2')
        self.publisher_ = self.create_publisher(String, 'topic1', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_pcl2)

        # dummy string data to send
        self.i = 0
        
    def publish_pcl2(self):
        msg = String()
        msg.data = f"Hello world: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

        self.i += 1
        


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
