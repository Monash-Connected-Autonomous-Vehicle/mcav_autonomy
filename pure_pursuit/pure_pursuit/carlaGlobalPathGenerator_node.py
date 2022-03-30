#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
import carla
from carla import Transform, Location, Rotation, World
from mcav_interfaces.msg import WaypointArray, Waypoint
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class WaypointPublisher(Node):


    def __init__(self):
        super().__init__('waypoint_publisher')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.waypoints_pub = self.create_publisher(WaypointArray, '/global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.spinner = self.create_timer(timer_period, self.waypoints_callback)
        
        # waypoint generation arguments
        self.debug = None
        self.map = None
        self.vehicle = None
        self.vehicle_pos = None
        self.path = None
        self.length_path = 143 # variable
        self.target_vel = 3.0 # target velocity (m/s)


    # Main callback function for publishing waypoints
    def waypoints_callback(self):
        self.vehicle_pos = self.vehicle.get_transform() # record vehicle pose on each loop
        wp_list_msg = WaypointArray()
        wp_list_msg.waypoints, pose_msg = self.waypoints2msg(self.path) # converts to WaypointArray message
        self.draw_waypoints(self.path, carla.Color(0, 0, 255)) # highlight waypoint path blue
        (self.waypoints_pub).publish(wp_list_msg) 
        (self.pose_pub).publish(pose_msg)


    # Generates waypoint path
    def gen_waypoints(self):      
        wp_start = (self.map).get_waypoint(self.vehicle.get_transform().location) # generates a waypoint at vehicle start location
        wp_next = wp_start
        wp_list = [None]*(self.length_path)
        
        for i in range(len(wp_list)):
            wp_list[i] = wp_next
            wp = wp_next.next(2)
            wp_next = wp[0]

        return wp_list
    
    
    # Converts waypoint to Waypoint message
    def waypoints2msg(self, waypoints):
        pose_msg = PoseWithCovarianceStamped()
#        pose_msg.pose.pose.position.x = self.vehicle_pos.location.x
#        pose_msg.pose.pose.position.y = self.vehicle_pos.location.y
#        pose_msg.pose.pose.position.z = self.vehicle_pos.location.z 
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = self.vehicle_pos.rotation.yaw*(math.pi/180)
        wp_msg_list = [None]*len(waypoints)
        
        for i in range(len(wp_msg_list)):
            wp_msg = Waypoint()
            
            v_yaw = self.vehicle_pos.rotation.yaw
            v_x = self.vehicle_pos.location.x
            v_y = self.vehicle_pos.location.y
            wp_x = waypoints[i].transform.location.x
            wp_y = waypoints[i].transform.location.y
            wp_z = waypoints[i].transform.location.z
            wp_yaw = waypoints[i].transform.rotation.yaw
            
            # transform waypoint coordinates to vehicle frame
            rad_factor = math.pi/180
            wp_msg.pose.position.x = ((wp_x-v_x)*math.cos(-v_yaw*rad_factor)
                                     -(wp_y-v_y)*math.sin(-v_yaw*rad_factor))
            wp_msg.pose.position.y = ((wp_x-v_x)*math.sin(-v_yaw*rad_factor)
                                     +(wp_y-v_y)*math.cos(-v_yaw*rad_factor))
            wp_msg.pose.position.z = wp_z
            wp_msg.pose.orientation.z = wp_yaw*(math.pi/180)
            
            # waypoint velocity is zero upon reaching last waypoint
            if i == len(wp_msg_list)-1:
                wp_msg.velocity.linear.x = 0.0
            else:
                wp_msg.velocity.linear.x = self.target_vel
                
            wp_msg_list[i] = wp_msg  

        return wp_msg_list, pose_msg
        
        
    # Visualises waypoint paths    
    def draw_waypoints(self, wp_list, color):
        for i in wp_list:
            (self.debug).draw_point(i.transform.location + carla.Location(z=0.25), 0.05, color, False)


def main(args=None):
    world = None
    
    #Node initialisation
    rclpy.init(args=args)
    wp_pub = WaypointPublisher()

    try:
        # create a client to communicate with the server
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        vehicle = (world.get_actors())[0]	
		       
        #set node parameters and spin
        wp_pub.debug = world.debug
        wp_pub.map = world.get_map()
        wp_pub.vehicle = vehicle
        wp_pub.path = wp_pub.gen_waypoints()
        rclpy.spin(wp_pub)

    except KeyboardInterrupt:
        # https://github.com/carla-simulator/carla/issues/1346
        print("User Cancelled")
    finally:
        print("shutting down node")
        wp_pub.destroy_node()
        rclpy.shutdown()
        print("done.")


if __name__ == '__main__':
    main()
