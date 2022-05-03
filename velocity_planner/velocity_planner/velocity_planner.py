from tarfile import LENGTH_NAME
import rclpy
import logging
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from mcav_interfaces.msg import WaypointArray, Waypoint, DetectedObjectArray
import numpy as np
import math
import tf2_ros
from velocity_planner.vector_ops import q_normalise, qv_mult


class VelocityPlanner(Node):

    def __init__(self):
        super().__init__('velocity_planner')

        # Subscriber
        timer_period = 0.01  # seconds
        self.spinner = self.create_timer(timer_period, self.spin)
        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
            'current_pose', self.initial_pose_callback, 10)
        self.initial_pose_sub  # prevent unused variable warning
        self.waypoints_sub = self.create_subscription(WaypointArray,
            'global_waypoints', self.waypoints_callback, 10)
        self.waypoints_sub  # prevent unused variable warning
        self.objects_sub = self.create_subscription(DetectedObjectArray,
            'detected_objects', self.objects_callback, 10)
        self.objects_sub  # prevent unused variable warning

        # Publisher
        self.global_wp_bl_pub = self.create_publisher(WaypointArray, 'global_baselink_waypoints', 10)
        self.local_wp_bl_pub = self.create_publisher(WaypointArray, 'local_baselink_waypoints', 10)
        self.local_wp_map_pub = self.create_publisher(WaypointArray, 'local_map_waypoints', 10)
        

        self.declare_parameter('max_velocity', 5.6) # maximum waypoint velocity used for speed capping
        self.declare_parameter('local_plan_max_length', 25) # number of waypoints to plan ahead
        self.declare_parameter('max_acceleration', 0.5) # m/s/waypoint
        self.declare_parameter('obj_waypoint_distance_threshold', 1.5) # if an object is within this distance of a path,
        # it will be considered as blocking the path
        self.declare_parameter('obj_stopping_waypoint_count', 3) # number of waypoints before object to stop at

        # Initialise tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.position = np.array([])
        self.yaw = None
        self.global_waypoints = []
        self.global_wp_coords = np.array([])
        self.objects = []
        
        self.get_logger().set_level(logging.DEBUG)


    def initial_pose_callback(self, pose_msg: PoseWithCovarianceStamped):
        self.position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        self.yaw = pose_msg.pose.pose.orientation.z
        
        
    def waypoints_callback(self, msg: WaypointArray):
        # TODO: transform so that they can be in different coordinate systems
        self.global_waypoints = msg.waypoints
        self.global_wp_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in msg.waypoints])
        

    def objects_callback(self, msg: DetectedObjectArray):
        self.objects = msg.detected_objects


    def find_nearest_waypoint(self, waypoint_coords, position) -> int:
        """ Inputs: position 1x2 numpy array [x, y] """
        # Algorithm from https://codereview.stackexchange.com/a/28210
        deltas = waypoint_coords - position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)


    def find_object_waypoints(self, waypoints):
        """ Checks if path is blocked by an object. Path is considered blocked if the closest waypoint
        on the path to an object is within a distance threshold, and the object is in front of that waypoint,
        not behind. """
        waypoint_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in waypoints])
        stopping_indices = []
        distance_threshold = self.get_parameter('obj_waypoint_distance_threshold').get_parameter_value().double_value
        stopping_wp_count = self.get_parameter('obj_stopping_waypoint_count').get_parameter_value().integer_value

        for obj in self.objects:
            position = np.array([obj.pose.position.x, obj.pose.position.y])
            nearest_wp = self.find_nearest_waypoint(waypoint_coords, position)
            wp_obj_vec = position - waypoint_coords[nearest_wp, :]

            #self.get_logger().info(f" obj {obj.object_id} wp_vec {wp_obj_vec} pos {position} clusters.")
            distance_to_path = np.linalg.norm(wp_obj_vec)
            #self.get_logger().info(f"d2p {distance_to_path}")
            if distance_to_path < distance_threshold:
                stopping_index = max(nearest_wp-stopping_wp_count, 0)
                stopping_indices.append(stopping_index)

        return stopping_indices


    def slow_to_stop(self, waypoints, stopping_index):
        main_speed = max(wp.velocity.linear.x for wp in waypoints)
        max_accel = self.get_parameter('max_acceleration').get_parameter_value().double_value

        # Gradually slow to a stop with constant deceleration before the stopping waypoint
        slowing_wp_count = min(math.ceil(main_speed / max_accel), len(waypoints))
        slowed_waypoints = waypoints.copy()
        slowing_indices = range(stopping_index, max(-1, stopping_index-slowing_wp_count), -1)
        for i in slowing_indices:
            curr_speed = slowed_waypoints[i].velocity.linear.x
            slower_speed = (stopping_index - i)*max_accel
            slowed_waypoints[i].velocity.linear.x = min(slower_speed, curr_speed, main_speed)
        
        # Zero everything past the stopping waypoint
        for i in range(stopping_index, len(slowed_waypoints)):
            slowed_waypoints[i].velocity.linear.x = 0.0

        return slowed_waypoints


    def slow_at_curve(self, waypoints):
        """ Regulates velocity of waypoints at curves """
        mu = 0.2 
        g = 9.81
        slowed_waypoints = waypoints.copy()
        wp_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in waypoints])
        vel_cap = self.get_parameter('max_velocity').get_parameter_value().double_value # global velocity cap
        vel_max = vel_cap
        
        for i in range(len(waypoints) - 2):
            len_b = np.linalg.norm(wp_coords[i] - wp_coords[i+1])
            len_a = np.linalg.norm(wp_coords[i] - wp_coords[i+2])
            len_c = np.linalg.norm(wp_coords[i+1] - wp_coords[i+2])
            thet_bac = math.acos((len_b**2 + len_c**2 - len_a**2)/(2*len_b*len_c)) # cos rule
            curve = 2*math.sin(thet_bac)/len_a # Menger curvature
            vel_flipped = max(1.0/vel_cap, math.sqrt(curve/(mu*g))) # limiting maximum possible curve velocity to avoid exploding numbers
            vel_max = min(vel_max, 1.0/vel_flipped) # local maximum velocity given curvature of waypoints

        for wp in slowed_waypoints:
            wp.velocity.linear.x = min(vel_max, wp.velocity.linear.x)
            # print(vel_max)

        return slowed_waypoints


    def speed_cap(self, waypoints):
        """ Failsafe to cap the speed of the car """
        capped_waypoints = waypoints.copy()
        max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        for wp in capped_waypoints:
            wp.velocity.linear.x = min(max_velocity, wp.velocity.linear.x) # Velocity cap for pure pursuit
                         
        return capped_waypoints


    def convert2base(self, wp_list): # converts local waypoints to base_link frame
        local_wp_msg = WaypointArray()
        
        for wp in wp_list:
            wp_new = Waypoint()
            v_yaw = self.yaw
            v_x = self.position[0]
            v_y = self.position[1]
            wp_x = wp.pose.position.x
            wp_y = wp.pose.position.y
            
            # Transformation about the z-axis
            wp_new.frame_id = "ego_vehicle"
            wp_new.pose.position.x = ((wp_x-v_x)*math.cos(-v_yaw)-(wp_y-v_y)*math.sin(-v_yaw))
            wp_new.pose.position.y = ((wp_x-v_x)*math.sin(-v_yaw)+(wp_y-v_y)*math.cos(-v_yaw))
            wp_new.velocity.linear.x = wp.velocity.linear.x
            local_wp_msg.waypoints.append(wp_new)
            
        return local_wp_msg


    def spin(self):
        if len(self.position) > 0 and len(self.global_wp_coords) > 0:
            # Publishes global waypoints in the base_link frame
            global_wp_bl = self.convert2base(self.global_waypoints)
            self.global_wp_bl_pub.publish(global_wp_bl)

            nearest_index = self.find_nearest_waypoint(self.global_wp_coords, self.position)

            # Speed cap
            self.global_waypoints = self.speed_cap(self.global_waypoints)

            # Stop at the end of the global waypoints
            slowed_global = self.slow_to_stop(self.global_waypoints, len(self.global_waypoints)-1)
            
            # Extract up to local_plan_max_length waypoints as the local plan
            local_plan_max_length = self.get_parameter('local_plan_max_length').get_parameter_value().integer_value
            final_wp_index = min(len(self.global_waypoints)-1, nearest_index + local_plan_max_length - 1)
            local_waypoints = slowed_global[nearest_index:final_wp_index+1]

            # Regulates velocity of waypoints at curves
            local_waypoints = self.slow_at_curve(local_waypoints)

            # Publishes local waypoints in the map frame
            local_map_wp_msg = WaypointArray()
            local_map_wp_msg.waypoints = local_waypoints
            self.local_wp_map_pub.publish(local_map_wp_msg)

            # Stop for the first detected object that blocks path
            local_wp_msg = self.convert2base(local_waypoints)
            stopping_indices = self.find_object_waypoints(local_wp_msg.waypoints)
            if len(stopping_indices) > 0:
                local_waypoints = self.slow_to_stop(local_waypoints, min(stopping_indices))

            # Publishes local waypoints in the base_link frame
            self.local_wp_bl_pub.publish(local_wp_msg)


def main(args=None):
    rclpy.init(args=args)

    planner = VelocityPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()