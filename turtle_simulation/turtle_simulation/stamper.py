import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
#import turtlesim Pose msg type
from turtlesim.msg import Pose as TurtlePose
class PoseStamper(Node):
    """Stamp Pose and convert from TurtlePose, unstamp Twist
    """
    def __init__(self):
        super().__init__('pose_stamper')
        #this subscribes to Turtle Sim pose
        self.pose_sub = self.create_subscription(TurtlePose, "pose_topic", self.pose_callback, 10)
        self.twist_sub = self.create_subscription(TwistStamped, "twist_stamped_topic", self.twist_callback, 10)
        self.twist_pub = self.create_publisher(Twist, "twist_topic", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "pose_stamped_topic", 10)

    def pose_callback(self, turtle_pose):
        msg = PoseStamped()
        msg.header.frame_id = "turtle_base"
        msg.header.stamp = self.get_clock().now().to_msg()
        pose = Pose()
        # Geometry pose describes position and orientation
        pose.position.x = turtle_pose.x
        pose.position.y = turtle_pose.y

        pose.orientation.x = math.cos(turtle_pose.theta)
        pose.orientation.y = math.sin(turtle_pose.theta)

        msg.pose = Pose() 
        
        self.pose_pub.publish(msg)
        # self.get_logger().info(f"Publishing pose to {self.subscriber.topic}")

    def twist_callback(self, msg):
        self.twist_pub.publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    stamper = PoseStamper()
    rclpy.spin(stamper)
    stamper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()