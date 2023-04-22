#!/usr/bin/env python
""" Simulates the StreetDrone by applying the desired actuation commands to a kinematic bicycle model and publishing
a transformation to the base_link frame """

import rclpy
from rclpy.node import Node
import tf2_ros
from sd_msgs.msg import SDControl
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseStamped
from geometry_msgs.msg import TransformStamped, TwistStamped
from tf2_ros import TransformBroadcaster

import numpy as np
import logging


class SimpleSim(Node):
    def __init__(self):
        super().__init__('simple_sim')

        # Spin the simulation at regular intervals
        self.timer_period = 0.1  # seconds
        self.spinner = self.create_timer(self.timer_period, self.spin)

        # Subscribers
        self.control_cmd = self.create_subscription(SDControl,
            'sd_control', self.control_callback, 10)
        self.control_cmd  # prevent unused variable warning
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.initialpose_sub

        # Publishers
        self.current_pose_pub = self.create_publisher(PoseStamped, 'current_pose', 10)
        self.current_vel_pub = self.create_publisher(TwistStamped, 'current_velocity', 10)
        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters
        self.declare_parameter('fixed_frame', 'map') # fixed coordinate frame relative to which we publish the vehicle position
        self.declare_parameter('vehicle_frame', 'base_link') # frame of the vehicle position

        # Initialise tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # Latest received control commands
        self.current_torque_nm = 0.0 
        self.current_steer_angle_rad = 0.0 # radians, left is positive 

        # Constants for converting SDControl message values (percentages) to Nm and radians
        # For now, these were chosen arbitrarily
        # TODO: determine the actual constants that correspond to the real-world vehicle
        self.TORQUE_PERCENT_TO_NM = 1.5 # Nm / %
        self.STEER_PERCENT_TO_RAD = 0.005 # Rad / %.

        # Kinematic bicycle model state
        self.state_x = 0.0 # metres
        self.state_y = 0.0 # metres
        self.state_phi = 0.0 # radians
        self.state_v = 0.0 # m/s
        # Vehicle parameters
        self.vehicle_mass_kg = 500.0 # approximate weight of the Renault Twizy is 500kg
        
        self.get_logger().set_level(logging.DEBUG)
        
        self.prev_time = self.get_clock().now()

    def control_callback(self, msg: SDControl):
        # Update the stored control command values

        # From SDControl message definition:
        # msg.torque: Range -100 to 100, -100 is max brake, +100 is max throttle
        # msg.steer: Range -100 to +100, +100 is maximum left turn
        # self.get_logger().info(f"Received command {msg.torque}, {msg.steer}")

        self.current_torque_nm = msg.torque * self.TORQUE_PERCENT_TO_NM
        self.current_steer_angle_rad = msg.steer * self.STEER_PERCENT_TO_RAD

        # self.get_logger().info(f"Steer angle is {self.current_steer_angle_rad*180/np.pi}")

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """ Sets current pose to the received pose, resets velocity """
        self.state_x = msg.pose.pose.position.x
        self.state_y = msg.pose.pose.position.y
        self.state_v = 0
        self.state_phi = 0 # TODO: update this based on received orientation

        self.get_logger().info("Received initialpose")

    def spin(self):
        # Update tf based on current commanded torque and steering after one time step
        
        elapsed_time_ns = (self.get_clock().now() - self.prev_time).nanoseconds
        elapsed_time = elapsed_time_ns / 1e9
        self.prev_time = self.get_clock().now()

        # Bicycle kinematic model as defined e.g. 
        # https://thef1clan.com/2020/09/21/vehicle-dynamics-the-kinematic-bicycle-model/
        # https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html
        # R. Rajamani, Vehicle Dynamics and Control: https://edisciplinas.usp.br/pluginfile.php/5349444/mod_resource/content/3/Rajesh_Rajamani_Vehicle_Dynamics_and_Con.pdf
        # page 27
        lf = 1.0 # distance from centre of gravity to front axle TODO: find real value
        lr = 0.5 # distance from centre of gravity to rear axle TODO: find real value

        # Calculate values for kinematic model
        # assume delta_r is 0 (front wheel steering) --> self.current_steer_angle_rad = delta_f
        beta = np.arctan((lr/(lf+lr))*np.tan(self.current_steer_angle_rad)) # "vehicle slip angle"
        x_dot = self.state_v * np.cos(self.state_phi+beta)
        y_dot = self.state_v * np.sin(self.state_phi+beta)
        phi_dot = self.state_v*np.cos(beta)*np.tan(self.current_steer_angle_rad)/(lr+lf) # yaw angle (global)
        #self.get_logger().info(f"timeelapsed: {elapsed_time}, phidot {phi_dot}, steerangle: {self.current_steer_angle_rad}, beta: {beta}, v: {self.state_v}")
        v_dot = self.current_torque_nm / self.vehicle_mass_kg

        # Integrate over time step
        self.state_phi += phi_dot*elapsed_time
        self.state_v += v_dot*elapsed_time
        self.state_x += x_dot*elapsed_time
        self.state_y += y_dot*elapsed_time

        # Get frame id parameters
        vehicle_frame = self.get_parameter('vehicle_frame').get_parameter_value().string_value
        fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        # Create this new tf
        t = TransformStamped() # Create an empty message
        # Transform is from fixed frame to vehicle frame (e.g. map to base_link)
        t.header.frame_id = fixed_frame
        t.child_frame_id = vehicle_frame
        # Set transform translation to msg pose position/rotation
        t.transform.translation.x = self.state_x
        t.transform.translation.y = self.state_y
        t.transform.rotation = z_angle_to_quat(self.state_phi+beta)

        # Broadcast tf
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

        # Publish current pose
        new_pose_msg = PoseStamped()
        new_pose_msg.header.frame_id = fixed_frame
        new_pose_msg.pose.position.x = self.state_x
        new_pose_msg.pose.position.y = self.state_y
        new_pose_msg.pose.orientation = z_angle_to_quat(self.state_phi+beta)
        self.current_pose_pub.publish(new_pose_msg)

        # Publish current velocity
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = vehicle_frame
        vel_msg.twist.linear.x = self.state_v
        vel_msg.twist.angular.z = phi_dot
        self.current_vel_pub.publish(vel_msg)

        # self.get_logger().info(f"v: {self.state_v:.2f}, phi: {self.state_phi:.2f}")

def z_angle_to_quat(angle):
    return rpy_to_quat(0.0, 0.0, angle)


# Adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_code
def rpy_to_quat(roll, pitch, yaw): # roll (x), pitch (Y), yaw (z)
    # Abbreviations for the various angular functions

    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def main(args=None):
    rclpy.init(args=args)
    simple_sim = SimpleSim()
    rclpy.spin(simple_sim)
    simple_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()