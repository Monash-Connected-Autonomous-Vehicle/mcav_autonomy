#!/usr/bin/env python3

import numpy as np
import math
import rclpy
from rclpy.node import Node
from mcav_interfaces.msg import WaypointArray
from geometry_msgs.msg import TwistStamped, PointStamped
import tf2_ros
import velocity_planner.transforms as transforms
import transforms3d as tf3d
from typing import Tuple
  
class PurePursuitNode(Node):
    """
    Node Class - subscribes to waypoints; publishes twist commands.
    """

    def __init__(self):
        super().__init__('purepursuit')

        self.waypoints = []
        self.Lfc = None    # [m] default look-ahead distance
        self.stop = False  # stopping flag
        self.failsafe_thresh = 2.8  # positive change in velocity at which the car will stop at
        self.vehicle_frame_id = "base_link"

        # Subscribers
        self.wp_subscriber = self.create_subscription(WaypointArray, 'local_waypoints', self.waypoints_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.wp_subscriber  # prevent 'unused variable' warning

        # Initialise tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.target_publisher = self.create_publisher(PointStamped, 'lookahead_point', 1)
        self.pp_publisher = self.create_publisher(TwistStamped, '/twist_cmd', 1)
        self.timer_period = 0.01
        self.spinner = self.create_timer(self.timer_period, self.spin)

        self.get_logger().info("Pure_pursuit node - Active")
    
    
    def waypoints_callback(self, wp_msg: WaypointArray):
        """Waypoint subscriber callback function.

        Args:
            wp_msg (WaypointArray): array of waypoints of type Waypoint
        """
        default_lookahead = 2.0
        self.waypoint_frame_id = wp_msg.frame_id
        self.waypoints = wp_msg.waypoints
        # shorten the lookahead distance when slow (determined experimentally)
        is_low_velocity = False
        if len(wp_msg.waypoints) > 0:
            linear_vel = self.waypoints[0].velocity.linear.x
            is_low_velocity = linear_vel < 4.0
            if is_low_velocity:
                self.Lfc = default_lookahead
            else:
                self.Lfc = 4.0 * linear_vel - 8.0 # formula determined experimentally for Tesla Model 3 in CARLA
                # self.Lfc = 11.028*linear_vel - 63.5  # Nissan Micra
        else:
            self.Lfc = default_lookahead
    

    def spin(self):   
        """
        Twist command publisher callback function.
        """
        if self.waypoints: 
            twist_msg = TwistStamped()

            if self.stop:
                self.get_logger().info("Stopping vehicle...")
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
            else:
                vlinear, v_angular = self.purepursuit()
                twist_msg.twist.linear.x = vlinear
                twist_msg.twist.angular.z = v_angular
                
            self.pp_publisher.publish(twist_msg)
            # self.get_logger().info(f"Stop: {self.stop}, Linear vel: {twist_msg.twist.linear.x:5.3f}, Angular vel: {twist_msg.twist.angular.z:5.3f}")
        else:
            self.get_logger().warning(f"Waiting for local waypoints")

    def publish_lookahead_point(self, tx, ty):
        """
        Description: 
        This function publishes the location of the lookahead point for visualisation/debugging.
        It looks up the transform between the waypoint frame and the vehicle frame using a 
        time-stamped message. The transformed point is then published to the target_publisher. 
        
        Args: tx x coordinate of the lookahead point.
              ty y coordinate of the lookahead point.
        
        """
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame=self.waypoint_frame_id,
                source_frame=self.vehicle_frame_id,
                time=rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().error(
                f'Could not transform {self.vehicle_frame_id} to {self.waypoint_frame_id}: {ex}')
            return
        
        target_msg = PointStamped()
        target_msg.header.frame_id = self.waypoint_frame_id
        tf_mat = transforms.tf_to_homogenous_mat(t)
        target_transformed = tf_mat @ np.array([tx, ty, 0.0, 1.0])
        target_msg.point.x = target_transformed[0]
        target_msg.point.y = target_transformed[1]
        self.target_publisher.publish(target_msg)

    def purepursuit(self) -> Tuple[float, float]:
        """
        Description: This function implements the pure pursuit algorithm to calculate the linear 
        and angular velocity of a vehicle. It first calls the target_searcher function to get the 
        coordinates of the target point, then calls the steer_control function to calculate the 
        steering angle, which is used to calculate the angular velocity. Finally, it returns the 
        linear velocity and angular velocity as a tuple.

        Returns: Tuple[float, float]: linear velocity and angular velocity
        
        """

        tx, ty, vlinear = self.target_searcher()
        self.publish_lookahead_point(tx, ty)
        gamma = self.steer_control(tx, ty)
        v_angular = vlinear*gamma
        
        return vlinear, v_angular


    def convert2base(self, wp_list): # converts given waypoints to base_link frame
        """
        Description: This function converts the given waypoints to the base_link frame. 
        It uses the transformation buffer to lookup the transform between the vehicle_frame_id 
        and the waypoint_frame_id, and then applies the transformation to the waypoints to 
        convert them to the vehicle coordinate frame (base_link frame). Finally, it returns 
        the converted x and y coordinates of the waypoints as a numpy array.

        Args:
        wp_list: Array containing a list of waypoints in the waypoint_frame_id frame.

        Returns: (wp_x, wp_y) (tuple): Tuple containing numpy arrays of x and y coordinates of the 
        converted waypoints in the vehicle_frame_id frame.

        """

        # Wait until a transform is available from tf
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame=self.vehicle_frame_id,
                source_frame=self.waypoint_frame_id,
                time=rclpy.time.Time()
            )

        except tf2_ros.TransformException as ex:
            self.get_logger().error(
                f'Could not transform {self.vehicle_frame_id} to {self.waypoint_frame_id}: {ex}')

            return

        wp_x = []
        wp_y = []

        for wp in wp_list:
            # Transform waypoint to vehicle coordinate frame
            vehicle_matrix = transforms.tf_to_homogenous_mat(t)
            wp_matrix = transforms.pose_to_homogenous_mat(wp.pose)
            new_wp_matrix = np.dot(vehicle_matrix, wp_matrix)

            pose = transforms.homogenous_mat_to_pose(new_wp_matrix)
            
            wp_x.append(pose.position.x)
            wp_y.append(pose.position.y)
            
        return (np.array(wp_x), np.array(wp_y))

            
    def target_searcher(self) -> Tuple[float, float]:
        """Interpolates target point in waypoint path at a lookahead distance away from vehicle.

        Returns:
             Tuple[float, float]: x and y coordinates of target
        """
        (wp_x, wp_y) = self.convert2base(self.waypoints)
        wp_d = np.hypot(wp_x, wp_y)
        
        if (wp_d >  self.Lfc).any() and (wp_d <  self.Lfc).any():
            points_within = wp_d <= self.Lfc
            points_beyond = wp_d > self.Lfc
            
            # coordinates of furthest point within lookahead range
            L_less = wp_d[points_within]  # distance of points within lookahead range
            d_prev_i = np.argmax(L_less)
            x1 = (wp_x[points_within])[d_prev_i]
            y1 = (wp_y[points_within])[d_prev_i]
            # coordinates of closest point beyond lookahead range
            L_more = wp_d[points_beyond]  # distance of points beyond lookahead range
            d_next_i = np.argmin(L_more)
            x2 = (wp_x[points_beyond])[d_next_i]
            y2 = (wp_y[points_beyond])[d_next_i]

            vlinear = self.waypoints[d_next_i].velocity.linear.x
            
            # maths to get coordinates of lookahead point
            ## linear line coeficients
            gradient = (y2-y1)/(x2-x1)
            y_inter = y1-gradient*x1
            ## coeficients of line intercept quadratic
            ### i.e. rearranging L**2 = y**2 + x**2, where y = mx + c
            a = gradient**2 + 1
            b = 2*gradient*y_inter
            c = y_inter**2 - self.Lfc**2
            ## quadratic formula to find intercept point, hence target coordinates
            tx_1 = (-b + math.sqrt(b**2 - 4*a*c))/(2*a)
            tx_2 = (-b - math.sqrt(b**2 - 4*a*c))/(2*a)
            
            if x1 < tx_1 < x2:  # this assumes x1 will always be closer than x2
                tx = tx_1
            else:
                tx = tx_2
            ty = gradient*tx + y_inter
        
        elif (wp_d <  self.Lfc).any(): 
            # target lookahead point in the trajectory of last waypoint if there are no points beyond lookahead distance
            bearing = math.atan2(wp_x[-1],wp_y[-1])
            tx = self.Lfc*math.sin(bearing)
            ty = self.Lfc*math.cos(bearing)
            vlinear = 0.0

        elif (wp_d >  self.Lfc).any():
            # target first point if there are no points before lookahead distance
            tx = wp_x[0]
            ty = wp_y[0]
            vlinear = self.waypoints[0].velocity.linear.x
        
        # self.get_logger().info(f"x: {tx:5.3f} , y: {ty:5.3f}")
        
        return tx, ty, vlinear  
    
    
    def steer_control(self, x: float, y: float) ->  float:
        """Calculates curvature of trajectory to target point.

        Args:
            x (float): x coordinate of target waypoint
            y (float): y coordinate of target waypoint

        Returns:
            gamma (float): curvature to target
        """
        gamma = 2.0*y/(self.Lfc**2)  # gamma = 1/r
        return gamma


# Main function
def main(args = None):
    rclpy.init(args=args)
    ppnode = PurePursuitNode()
    rclpy.spin(ppnode)
    ppnode.stop = True
    ppnode.twist_callback()
    ppnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()