# Where is this getting used?
import rclpy
import numpy as np
from rclpy.node import Node
from mcav_interfaces.msg import Waypoint, WaypointArray

class FakeWaypoints(Node):
    def __init__(self):
        super().__init__('fake_waypoint_publisher')
        self.publisher_ = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.init_waypoints()
        self.get_logger().info('Publishing "%d" waypoints' % len(self.waypoints))


    def init_waypoints(self):
        """ Modifies self.waypoints """
        x = np.arange(-5.0, 10.0, 0.5)
        y = np.zeros(len(x))

        for i in range(len(x)):
            waypoint = Waypoint()
            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            waypoint.velocity.linear.x = 5.5 # m/s
            self.waypoints.append(waypoint)

    def timer_callback(self):
        msg = WaypointArray()
        msg.waypoints = self.waypoints
        msg.frame_id = "map"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    fake_waypoint_publisher = FakeWaypoints()
    rclpy.spin(fake_waypoint_publisher)
    fake_waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
