#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import rclpy
import time as tim
from geometry_msgs.msg import Twist
from rclpy.node import Node
from mcav_interfaces.msg import WaypointArray, DetectedObjectArray, Waypoint

# Parameters
Lfc = 2.0  # [m] look-ahead distance
WB = 2.9  # [m] wheel base of vehicle


# Node Class - subscribes to waypoints; publishes twist commands    
class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('purepursuit_node')
        self.pp_subscriber = self.create_subscription(
                             WaypointArray, '/vel_waypoints', 
                             self.subscriber_callback, 10)
        self.pp_subscriber # prevent unused variable warning
        self.pp_publisher = self.create_publisher(Twist, '/carla/ego_vehicle/twist', 10)
        self.timer_period = 0.01
        self.spinner = self.create_timer(self.timer_period, self.publisher_callback)
        
        self.waypoints = []
    
    
    def subscriber_callback(self, wp_msg):
        self.waypoints = wp_msg
    
    
    def publisher_callback(self):   
        if self.waypoints: 
            v_linear, v_angular = self.purepursuit()
            twist_msg = Twist()
            twist_msg.linear.x = v_linear
            twist_msg.angular.x = v_angular
            self.pp_publisher.publish(twist_msg)
            print("published twist message")

    
    # Main pure pursuit callback function
    def purepursuit(self):
        tx, ty = self.target_searcher(Lfc) # finds local coordinate of target point
        gamma = self.steer_control(tx, ty, Lfc) # finds curvature of lookahead arc
        
        v_linear = (self.waypoints).waypoints[0].velocity.linear.x
        v_angular = v_linear*gamma
        
        return v_linear, v_angular


    def target_searcher(self, L):
        wp_x = np.array([wp.pose.position.x for wp in (self.waypoints).waypoints])
        wp_y = np.array([wp.pose.position.y for wp in (self.waypoints).waypoints])
        wp_d = np.hypot(wp_x, wp_y)
        
        L_less = wp_d[wp_d <= L] # distance of points within lookahead range
        L_more = wp_d[wp_d >  L] # distance of points beyond lookahead range
        d_prev_i = np.argmax(L_less)
        d_next_i = np.argmin(L_more)
        
        # coordinates of closest point within lookahead range
        x1 = (wp_x[wp_d <= L])[d_prev_i]
        y1 = (wp_y[wp_d <= L])[d_prev_i]
        # coordinates of closest point beyond lookahead range
        x2 = (wp_x[wp_d >  L])[d_next_i]
        y2 = (wp_y[wp_d >  L])[d_next_i]
#        print(y1, ", ", y2)
        
        # maths to get coordinates of lookahead point
        ## linear line coeficients
        m = (y2-y1)/(x2-x1)
        c = y1-m*x1
        ## coeficients of line intercept quadratic
        ### i.e. rearranging L**2 = y**2 + x**2, where y = mx + c
        a = m**2 + 1
        b = 2*m*c
        c = c**2 - L**2
        ## quadratic formula to find intercept point, hence target coordinates
        tx_1 = (-b + math.sqrt(b**2 - 4*a*c))/(2*a)
        tx_2 = (-b - math.sqrt(b**2 - 4*a*c))/(2*a)
        if x1 < tx_1 < x2:
            tx = tx_1
            ty = m*tx + c
        else:
            tx = tx_2
            ty = m*tx + c
            
        return tx, ty        
    
    
    def steer_control(self, x, y, L):
        gamma = 2*y/L
        
        return gamma


# Main function
def main(args = None):
    rclpy.init(args=args)
    ppnode = PurePursuitNode()
    rclpy.spin(ppnode)
    ppnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
