# This is a small node that accepts a TwistStamped message, and republishes
# the same values in a Twist message, without the stamp. This allows for
# interoperability of our stack that uses TwistStamped messages
# (SD-VehicleInterface and pure_pursuit), with CARLA and gazebo, which use
# Twist messages.
# It is best use via a launch file that uses "remap" to specify the input and
# output topics.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class StampRemover(Node):

    def __init__(self):
        super().__init__('twist_stamp_remover')
        self.stamped_sub = self.create_subscription(TwistStamped,
            'twist_stamped_input', self.twist_callback, 10)
        self.unstamped_pub = self.create_publisher(Twist, 'twist_unstamped_output', 0)
        # TODO: qos

    def twist_callback(self, msg):
        unstamped = msg.twist
        self.unstamped_pub.publish(unstamped)

def main(args=None):
    rclpy.init(args=args)
    stamp_remover = StampRemover()
    rclpy.spin(stamp_remover)
    stamp_remover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
