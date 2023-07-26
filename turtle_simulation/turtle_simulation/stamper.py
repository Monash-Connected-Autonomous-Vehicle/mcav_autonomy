import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped
#import turtlesim Pose msg type
from turtlesim.msg import Pose as TurtlePose
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

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
        pose.position.z = 0.0
        self.get_logger().info(f"Turtle pos: {turtle_pose.x},{turtle_pose.y}")

        q = quaternion_from_euler(0,0,turtle_pose.theta)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        msg.pose = pose
        
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