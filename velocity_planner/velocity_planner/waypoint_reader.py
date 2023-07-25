""" Node to read waypoints in from a csv file and publish them """
import rclpy
from rclpy.node import Node
import sys
import csv
from ament_index_python.packages import get_package_share_directory
import os

from mcav_interfaces.msg import Waypoint, WaypointArray

class WaypointReader(Node):

    def __init__(self):
        super().__init__('waypoint_reader')
        self.publisher_ = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.waypoints = []
        self.declare_parameter('waypoints_file', "town01_path.csv")
        wp_filename = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.init_waypoints(wp_filename)
        

    def init_waypoints(self, waypoints_file):
        """ Reads in waypoints from the given csv file and stores them """
        paths_csv_directory = os.path.join(get_package_share_directory('velocity_planner'))

        try:
            filename = os.path.join(paths_csv_directory, waypoints_file)
            with open(filename) as csvfile:
                csv_reader = csv.DictReader(csvfile, delimiter=',')

                for row in csv_reader:
                    waypoint = Waypoint()
                    waypoint.pose.position.x = float(row['x'])
                    waypoint.pose.position.y = float(row['y'])
                    waypoint.velocity.linear.x = float(row['velocity']) # m/s
                    try:
                        waypoint.lookahead = float(row['lookahead'])
                        #waypoint.lookahead = 5.0
                    except:
                        waypoint.lookahead = 5.0
                    self.waypoints.append(waypoint)

                self.get_logger().info('Publishing "%d" waypoints' % len(self.waypoints))

        except FileNotFoundError:
            self.get_logger().error(f"Error, waypoint file not found: {filename}")

    def timer_callback(self):
        msg = WaypointArray()
        msg.frame_id = "map"
        msg.waypoints = self.waypoints
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    waypoint_reader = WaypointReader()
    rclpy.spin(waypoint_reader)
    waypoint_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
