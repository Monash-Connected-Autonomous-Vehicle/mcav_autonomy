import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped


class PoseStamper(Node):
    def __init__(self):
        super().__init__('pose_stamper')
        self.subscriber = self.create_subscription(Pose, "pose_topic", self.pose_callback, 10)

        self.publisher = self.create_publisher(PoseStamped, "stamped_topic", 10)

    def pose_callback(self, pose_msg):
        msg = PoseStamped()
        msg.header.frame_id = "turtle_base"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose_msg
        self.publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    stamper = PoseStamper()
    rclpy.spin(stamper)
    stamper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()