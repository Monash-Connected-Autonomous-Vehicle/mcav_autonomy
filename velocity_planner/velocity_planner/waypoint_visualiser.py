import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from mcav_interfaces.msg import WaypointArray
import colorsys

class WaypointVisualiser(Node):

    def __init__(self):
        super().__init__('waypoint_visusaliser')

        self.global_sub = self.create_subscription(WaypointArray,
            'global_waypoints', self.global_callback, 10)
        self.local_sub = self.create_subscription(WaypointArray,
            'local_waypoints', self.local_callback, 10)

        self.vis_pub_ = self.create_publisher(MarkerArray, 'global_waypoint_markers', 0)
        self.local_vis_pub_ = self.create_publisher(MarkerArray, 'local_waypoint_markers', 0)

        self.declare_parameter('max_velocity', 5.6) # m/s

        # these keep track of the length of the visualisation marker arrays published
        # so that we can publish the correct number of DELETE markers
        self.local_waypoints_length = 0
        self.global_waypoints_length = 0

        self.text_scale = 0.3
        self.text_offset = self.text_scale * 3 # used to move text so it doesn't overlap waypoints
        self.text_offset_2 = self.text_scale * 6 # used to move text so it doesn't overlap waypoints

    def local_callback(self, msg):
        markers = []

        top_speed = self.get_parameter('max_velocity').get_parameter_value().double_value

        for index, waypoint in enumerate(msg.waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = msg.frame_id
            pose_marker.ns = 'local_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.SPHERE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.6
            pose_marker.scale.y = 0.6
            pose_marker.scale.z = 0.01
            pose_marker.color.a = 0.9 # Don't forget to set the alpha!

            # changes colour according to fraction of max speed
            # green->red gradient for fast->slow
            green_hue = 90
            speed_fraction = waypoint.velocity.linear.x/top_speed
            interpolated_hue = green_hue*speed_fraction/255
            (r, g, b) = colorsys.hsv_to_rgb(interpolated_hue, 1.0, 1.0)
            pose_marker.color.r = float(r)
            pose_marker.color.g = float(g)
            pose_marker.color.b = float(b)

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = msg.frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + self.text_offset 
            velocity_marker.scale.x = self.text_scale
            velocity_marker.scale.y = self.text_scale
            velocity_marker.scale.z = self.text_scale
            velocity_marker.color.a = 0.9 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            velocity_marker.color.r = float(r)
            velocity_marker.color.g = float(g)
            velocity_marker.color.b = float(b)

            markers.append(velocity_marker)

            """
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = msg.frame_id
            lookahead_marker.ns = 'velocity'
            lookahead_marker.id = index
            lookahead_marker.type = Marker.TEXT_VIEW_FACING
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = waypoint.pose.position.x
            lookahead_marker.pose.position.y = waypoint.pose.position.y + self.text_offset 
            lookahead_marker.scale.x = self.text_scale
            lookahead_marker.scale.y = self.text_scale
            lookahead_marker.scale.z = self.text_scale
            lookahead_marker.color.a = 0.9 # Don't forget to set the alpha!
            lookahead_marker.text = f"{waypoint.lookahead:.2f}"

            lookahead_marker.color.r = float(r)
            lookahead_marker.color.g = float(g)
            lookahead_marker.color.b = float(b)

            markers.append(lookahead_marker)
            """

        # Remove extra markers
        for index in range(len(msg.waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'local_waypoints'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(len(msg.waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'velocity'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)

        """
        for index in range(len(msg.waypoints), self.local_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'lookahead'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        """

        # update record of number of markers published so they can be deleted next time
        self.local_waypoints_length = len(msg.waypoints) 
        
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.local_vis_pub_.publish(marker_array)

    def global_callback(self, msg):
        markers = []
        for index, waypoint in enumerate(msg.waypoints):
            pose_marker = Marker()
            pose_marker.header.frame_id = msg.frame_id
            pose_marker.ns = 'global_waypoints'
            pose_marker.id = index
            pose_marker.type = Marker.CUBE
            pose_marker.action = Marker.ADD
            pose_marker.pose = waypoint.pose
            pose_marker.scale.x = 0.1
            pose_marker.scale.y = 0.1
            pose_marker.scale.z = 0.02
            pose_marker.color.a = 1.0 # Don't forget to set the alpha!
            pose_marker.color.r = 1.0
            pose_marker.color.g = 1.0
            pose_marker.color.b = 0.0

            markers.append(pose_marker)

            velocity_marker = Marker()
            velocity_marker.header.frame_id = msg.frame_id
            velocity_marker.ns = 'velocity'
            velocity_marker.id = index
            velocity_marker.type = Marker.TEXT_VIEW_FACING
            velocity_marker.action = Marker.ADD
            velocity_marker.pose.position.x = waypoint.pose.position.x
            velocity_marker.pose.position.y = waypoint.pose.position.y + self.text_offset
            velocity_marker.scale.x = self.text_scale
            velocity_marker.scale.y = self.text_scale
            velocity_marker.scale.z = self.text_scale
            velocity_marker.color.a = 0.9 # Don't forget to set the alpha!
            velocity_marker.text = f"{waypoint.velocity.linear.x:.2f}"

            velocity_marker.color.r = 1.0
            velocity_marker.color.g = 1.0
            velocity_marker.color.b = 0.0

            markers.append(velocity_marker)

            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = msg.frame_id
            lookahead_marker.ns = 'lookahead'
            lookahead_marker.id = index
            lookahead_marker.type = Marker.TEXT_VIEW_FACING
            lookahead_marker.action = Marker.ADD
            lookahead_marker.pose.position.x = waypoint.pose.position.x
            lookahead_marker.pose.position.y = waypoint.pose.position.y + self.text_offset_2
            lookahead_marker.scale.x = self.text_scale
            lookahead_marker.scale.y = self.text_scale
            lookahead_marker.scale.z = self.text_scale
            lookahead_marker.color.a = 0.9 # Don't forget to set the alpha!
            lookahead_marker.text = f"{waypoint.lookahead:.2f}"

            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 1.0
            lookahead_marker.color.b = 0.0

            markers.append(lookahead_marker)

        # Remove extra markers
        for index in range(self.global_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'global_waypoints'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(self.global_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'velocity'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)
        for index in range(self.global_waypoints_length):
            marker = Marker()
            marker.header.frame_id = msg.frame_id
            marker.ns = 'lookahead'
            marker.id = index
            marker.action = Marker.DELETE
            markers.append(marker)

        # update record of number of markers published so they can be deleted next time
        self.local_waypoints_length = len(msg.waypoints) 

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.vis_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    waypoint_visusaliser = WaypointVisualiser()
    rclpy.spin(waypoint_visusaliser)
    waypoint_visusaliser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
