#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import rclpy
import time as tim
from geometry_msgs.msg import Twist
from rclpy.node import Node
from mcav_interfaces.msg import WaypointArray, Waypoint


# Node Class - subscribes to waypoints; publishes twist commands    
class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('purepursuit')
        self.wp_subscriber = self.create_subscription(
                             WaypointArray, 'local_baselink_waypoints', 
                             self.wp_subscriber_callback, 1)
        self.wp_subscriber # prevent 'unused variable' warning
        self.pp_publisher = self.create_publisher(Twist, '/carla/ego_vehicle/twist', 1)
        self.timer_period = 0.01
        self.spinner = self.create_timer(self.timer_period, self.publisher_callback)
        
        self.waypoints = []
        self.Lfc = None  # [m] default look-ahead distance
    
    
    def wp_subscriber_callback(self, wp_msg):
        self.waypoints = wp_msg.waypoints
        v = self.waypoints[0].velocity.linear.x
        # Lookahead distance dependent on velocity
        self.Lfc = 4*v-8 if v > 2.75 else 3  # Tesla model3
        # self.Lfc = 11.028*v - 63.5  # Nissan Micra
 
    
    def publisher_callback(self):   
        if self.waypoints: 
            twist_msg = Twist()
            v_linear, v_angular = self.purepursuit()
            twist_msg.linear.x = v_linear
            twist_msg.angular.z = v_angular
            self.pp_publisher.publish(twist_msg)
            print("Linear vel: ", twist_msg.linear.x ,", Angular vel: ", twist_msg.angular.z)
#            print("published twist message")

    
    # Main pure pursuit callback function
    def purepursuit(self):
        tx, ty = self.target_searcher(self.Lfc) # finds local coordinate of target point
        gamma = self.steer_control(tx, ty, self.Lfc) # finds curvature of lookahead arc
        
        v_linear = self.waypoints[0].velocity.linear.x
        v_angular = v_linear*gamma
        return v_linear, v_angular


    # Searches for target point relating to lookahead distance
    def target_searcher(self, L):
        wp_x = np.array([wp.pose.position.x for wp in self.waypoints])
        wp_y = np.array([wp.pose.position.y for wp in self.waypoints])
        wp_d = np.hypot(wp_x, wp_y)
        
        if (wp_d >  L).any() and (wp_d <  L).any():
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
            
            # maths to get coordinates of lookahead point
            ## linear line coeficients
            gradient = (y2-y1)/(x2-x1)
            y_inter = y1-gradient*x1
            ## coeficients of line intercept quadratic
            ### i.e. rearranging L**2 = y**2 + x**2, where y = mx + c
            a = gradient**2 + 1
            b = 2*gradient*y_inter
            c = y_inter**2 - L**2
            ## quadratic formula to find intercept point, hence target coordinates
            tx_1 = (-b + math.sqrt(b**2 - 4*a*c))/(2*a)
            tx_2 = (-b - math.sqrt(b**2 - 4*a*c))/(2*a)
            
            if x1 < tx_1 < x2: # this assumes x1 will always be closer than x2
                tx = tx_1
            else:
                tx = tx_2
            ty = gradient*tx + y_inter
        elif (wp_d <  L).any(): 
            # target final point if there are no points beyond lookahead distance
            tx = wp_x[-1]
            ty = wp_y[-1]
        elif (wp_d >  L).any():
            # target first point if there are no points before lookahead distance
            tx = wp_x[0]
            ty = wp_y[0]        
        
        print("x: ", tx ,", y: ", ty)
        
        return tx, ty        
    
    
    def steer_control(self, x, y, L):
        gamma = 2*y/(L**2) # gamma = 1/r
        
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
