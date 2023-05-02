import math
import rclpy
import tf2_ros
import logging
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import velocity_planner.transforms as transforms
from mcav_interfaces.msg import WaypointArray, Waypoint, DetectedObjectArray, DetectedObject

class VelocityPlanner(Node):
    """
    VelocityPlanner plans the optimal velocity and trajectory for the StreetDrone to follow while 
    avoiding detected objects. The maximum velocity, maximum acceleration, distance threshold for 
    considering objects to be blocking the path, and number of waypoints to stop before an object
    is configurable parameters.
    
    Subscriptions:
    - "current_pose"    : the current pose of the StreetDrone (geometry_msgs/PoseStamped).
    - "global_waypoints": a list of waypoints to follow (mcav_interfaces/WaypointArray).
    - "detected_objects": a list of objects detected by the robot (mcav_interfaces/DetectedObjectArray).
    
    Publishers:
    - "local_waypoints" : a list of local waypoints to follow to the  topic (mcav_interfaces/WaypointArray).

    """

    def __init__(self):
        super().__init__('velocity_planner')
        # Subscribers
        timer_period = 0.05  # seconds
        self.spinner = self.create_timer(timer_period, self.spin)
        self.current_pose_sub = self.create_subscription(PoseStamped, 'current_pose', self.current_pose_callback, 10)
        self.waypoints_sub = self.create_subscription(WaypointArray,'global_waypoints', self.waypoints_callback, 10)
        self.objects_sub = self.create_subscription(DetectedObjectArray,'detected_objects', self.objects_callback, 10)

        # Prevent unused variable warning
        self.waypoints_sub
        self.objects_sub

        # Publishers
        self.local_wp_pub = self.create_publisher(WaypointArray, 'local_waypoints', 10)
        
        # Parameters (can be changed in launch file)
        self.declare_parameter('max_velocity', 0.1) # maximum waypoint velocity used for speed capping
        self.declare_parameter('local_plan_max_length', 25) # number of waypoints to plan ahead
        self.declare_parameter('max_acceleration', 0.5) # m/s/waypoint
        self.declare_parameter('obj_waypoint_distance_threshold', 2.0) # if an object is within this distance of a path,
        # it will be considered as blocking the path
        self.declare_parameter('obj_stopping_waypoint_count', 3) # number of waypoints before object to stop at
        self.declare_parameter('vehicle_frame_id', "base_link")

        # Initialise tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.position = np.array([])
        self.yaw = None
        self.global_waypoints = []
        self.global_wp_coords = np.array([])
        self.objects = []
        
        self.get_logger().set_level(logging.DEBUG)


    def current_pose_callback(self, pose_msg: PoseStamped):

        # Extracts position and orientation information.
        self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y])
        self.yaw = pose_msg.pose.orientation.z

        if len(self.position) == 0:
            self.get_logger().info("Received position")
        
        
    def waypoints_callback(self, msg: WaypointArray):
        # TODO: transform so that they can be in different coordinate systems ######### Is this done, why is this here?
        # Updates the global list of waypoints ("global_waypoints") to ######### which frame??. 

        if len(self.global_wp_coords) == 0:
            self.get_logger().info("Received global waypoints")

        self.global_waypoints = msg.waypoints
        self.waypoints_frame = msg.frame_id
        self.global_wp_coords = np.array([(wp.pose.position.x, wp.pose.position.y) for wp in msg.waypoints])
        

    def objects_callback(self, msg: DetectedObjectArray):
        # Convert objects to the map frame so everything is in the same frame
        self.objects = [self.transform_to_map(obj) for obj in msg.detected_objects]
    

    def transform_to_map(self, obj: DetectedObject):
        """ Applies a transformation to the pose of the object so that it is with respect to the map frame """

        # Wait until a transform is available from tf
        map_frame_id = "map"
        try:
            to_frame_rel = map_frame_id
            from_frame_rel = obj.frame_id
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            tf_matrix = transforms.tf_to_homogenous_mat(t)
            object_matrix = transforms.pose_to_homogenous_mat(obj.pose)
            new_object_matrix = np.dot(tf_matrix, object_matrix)
            new_object = obj
            new_object.frame_id = map_frame_id
            new_object.pose = transforms.homogenous_mat_to_pose(new_object_matrix)

            return new_object

        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')


    def find_nearest_waypoint(self, waypoint_coords, position) -> int:
        """ 
        The function calculates the Euclidean distance between the current position and 
        all waypoints and then returns the index of the waypoint with the smallest distance.
        Algorithm from https://codereview.stackexchange.com/a/28210

        Inputs: position 1x2 numpy array [x, y]

        Ouput: index of the waypoint with the smallest distance
                
        """
        deltas = waypoint_coords - position
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)


    def find_object_waypoints(self, waypoints):
        """
        Checks if path is blocked by an object. Path is considered blocked if the closest waypoint
        on the path to an object is within a distance threshold, and the object is in front of that waypoint,
        not behind. 
        
        """

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
            # self.get_logger().info(f"d2p {distance_to_path}")
            if distance_to_path < distance_threshold:
                stopping_index = max(nearest_wp-stopping_wp_count, 0)
                stopping_indices.append(stopping_index)

        return stopping_indices


    def slow_to_stop(self, waypoints, stopping_index):
        """
        Gradually slows down a list of waypoints with constant deceleration until a stopping index, 
        then sets velocities to zero past that index.

        Inputs:
        waypoints: a list of waypoints containing position and velocity information.
        stopping_index: an integer index indicating the point in waypoints where the vehicle should come to a stop
        
        Outputs:
        slowed_waypoints: a list of waypoints with adjusted velocity at curves

        """

        #### Yet to understand this function properly

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
        """
        This function regulates the velocity of waypoints at curves.

        Inputs:
        waypoints: a list of waypoints containing position and velocity information.
        
        Outputs:
        slowed_waypoints: a list of waypoints with adjusted velocity at curves
        
        """
        #### Yet to understand this function properly

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
            cos_theta = (len_b**2 + len_c**2 - len_a**2)/(2*len_b*len_c)
            if abs(cos_theta) > 1.0:
                # can occur due to floating point rounding errors
                # limit to +/- 180 degrees
                thet_bac = np.pi * np.sign(cos_theta) 
            else:
                thet_bac = math.acos((len_b**2 + len_c**2 - len_a**2)/(2*len_b*len_c)) # cos rule. angle between segments [i,i+1] and [i+1,i+2]
            curvature = abs(2*math.sin(thet_bac)/len_a) # Menger curvature
            vel_flipped = max(1.0/vel_cap, math.sqrt(curvature/(mu*g))) # limiting maximum possible curve velocity to avoid exploding numbers
            vel_max = min(vel_max, 1.0/vel_flipped) # local maximum velocity given curvature of waypoints

        for wp in slowed_waypoints:
            wp.velocity.linear.x = min(vel_max, wp.velocity.linear.x)

        return slowed_waypoints


    def speed_cap(self, waypoints):
        """ 
        This function is used to set a maximum speed limit for the vehicle's waypoints.

        Inputs: 
        waypoints: a list of waypoints (which contain the velocity of the vehicle) as an argument.

        Outputs: 
        capped_waypoints: a new list of waypoints with the velocities capped at a specified maximum speed limit.
        
        """

        capped_waypoints = waypoints.copy()
        max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        for wp in capped_waypoints:
            wp.velocity.linear.x = min(max_velocity, wp.velocity.linear.x) # Velocity cap for pure pursuit
                         
        return capped_waypoints

    def spin(self):
        if len(self.position) > 0 and len(self.global_wp_coords) > 0:
            nearest_index = self.find_nearest_waypoint(self.global_wp_coords, self.position)

            # Speed cap
            capped_speed = self.speed_cap(self.global_waypoints)

            # Stop at the end of the global waypoints
            slowed_global = self.slow_to_stop(capped_speed, len(capped_speed)-1)
            
            # Extract up to local_plan_max_length waypoints as the local plan
            local_plan_max_length = self.get_parameter('local_plan_max_length').get_parameter_value().integer_value
            final_wp_index = min(len(slowed_global)-1, nearest_index + local_plan_max_length - 1)
            local_waypoints = slowed_global[nearest_index:final_wp_index+1]

            # Regulates velocity of waypoints at curves
            local_waypoints = self.slow_at_curve(local_waypoints)

            # Stop for the first detected object that blocks path
            stopping_indices = self.find_object_waypoints(local_waypoints)
            if len(stopping_indices) > 0:
                local_waypoints = self.slow_to_stop(local_waypoints, min(stopping_indices))
            
            # Publish local waypoints in the map frame
            local_wp_msg = WaypointArray()
            local_wp_msg.frame_id = self.waypoints_frame
            local_wp_msg.waypoints = local_waypoints
            self.local_wp_pub.publish(local_wp_msg)

        # Log the reason that we cannot plan yet
        if len(self.position) == 0:
            self.get_logger().warn("Waiting for position")
        if len(self.global_wp_coords) == 0:
            self.get_logger().warn("Waiting for global waypoints")


def main(args=None):
    rclpy.init(args=args)

    planner = VelocityPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()