#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import rclpy
import time as tim
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose
from rclpy.node import Node
from mcav_interfaces.msg import WaypointArray, Waypoint


# Node Class - subscribes to waypoints; publishes twist commands    
class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('purepursuit')

        self.waypoints = []
        self.Lfc = None  # [m] default look-ahead distance
        self.v = None  # [m/s] linear velocity
        self.stop = False
        self.failsafe_thresh = 2.8

        # Subscribers
        self.wp_subscriber = self.create_subscription(
                             WaypointArray, 'local_baselink_waypoints', 
                             self.waypoints_callback, 1)
        self.wp_subscriber # prevent 'unused variable' warning
        
        # Publishers
        self.target_publisher = self.create_publisher(PoseWithCovarianceStamped, 'target_pose', 1)
        self.pp_publisher = self.create_publisher(Twist, '/carla/ego_vehicle/twist', 1)
        self.timer_period = 0.01
        self.spinner = self.create_timer(self.timer_period, self.twist_callback)
    
    
    def waypoints_callback(self, wp_msg):
        self.waypoints = wp_msg.waypoints
        self.v = self.waypoints[1].velocity.linear.x
        self.failsafe(self.waypoints[0].velocity.linear.x, self.waypoints[1].velocity.linear.x)
        
        # Lookahead distance dependent on velocity
        self.Lfc = 4*self.v-8 if self.v > 4.0 else 8.0  # Tesla model3
        # self.Lfc = 11.028*v - 63.5  # Nissan Micra
        # self.Lfc = 12.0
        # self.lookahead_regulator()
 
    
    def twist_callback(self):   
        if self.waypoints: 
            twist_msg = Twist()

            if self.stop:
                print("Stopping vehicle...")
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0               
            else:
                v_linear, v_angular = self.purepursuit()
                twist_msg.linear.x = v_linear
                twist_msg.angular.z = v_angular
                
            self.pp_publisher.publish(twist_msg)
            print("Linear vel: ", twist_msg.linear.x ,", Angular vel: ", twist_msg.angular.z)
#            print("published twist message")


    def lookahead_regulator(self):
        wp_x = np.array([wp.pose.position.x for wp in self.waypoints])
        wp_y = np.array([wp.pose.position.y for wp in self.waypoints])
        wp_yaw = np.array([abs(math.atan2(wp_y[i],wp_x[i])*180.0/math.pi) for i in range(len(self.waypoints))])
        # wp_yaw = np.array([wp.pose.orientation.z for wp in self.waypoints])
        avg_yaw = sum(wp_yaw)/len(wp_yaw)
        
        k = 1/200.0
        self.Lfc = self.Lfc*(1 + v*avg_yaw/360.0)
        print('avg_yaw: ', str(avg_yaw) + '     Lfc: ', str(self.Lfc))

        
    # Main pure pursuit callback function
    def purepursuit(self):
        tx, ty = self.target_searcher() # finds local coordinate of target point
        
        target_msg = PoseWithCovarianceStamped()
        target_msg.pose.pose.position.x = tx
        target_msg.pose.pose.position.y = ty
        self.target_publisher.publish(target_msg)

        gamma = self.steer_control(tx, ty) # finds curvature of lookahead arc
        v_linear = self.waypoints[1].velocity.linear.x
        v_angular = v_linear*gamma
        
        return v_linear, v_angular


    # Searches for target point relating to lookahead distance
    def target_searcher(self):
        wp_x = np.array([wp.pose.position.x for wp in self.waypoints])
        wp_y = np.array([wp.pose.position.y for wp in self.waypoints])
        wp_d = np.hypot(wp_x, wp_y)
        
        if (wp_d >  self.Lfc).any() and (wp_d <  self.Lfc).any():
            L_less = wp_d[wp_d <= self.Lfc] # distance of points within lookahead range
            L_more = wp_d[wp_d >  self.Lfc] # distance of points beyond lookahead range
            d_prev_i = np.argmax(L_less)
            d_next_i = np.argmin(L_more)
            
            # coordinates of closest point within lookahead range
            x1 = (wp_x[wp_d <= self.Lfc])[d_prev_i]
            y1 = (wp_y[wp_d <= self.Lfc])[d_prev_i]
            # coordinates of closest point beyond lookahead range
            x2 = (wp_x[wp_d >  self.Lfc])[d_next_i]
            y2 = (wp_y[wp_d >  self.Lfc])[d_next_i]
            
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
            
            if x1 < tx_1 < x2: # this assumes x1 will always be closer than x2
                tx = tx_1
            else:
                tx = tx_2
            ty = gradient*tx + y_inter
        elif (wp_d <  self.Lfc).any(): 
            # target lookahead point in the trajectory of last waypoint if there are no points beyond lookahead distance
            bearing = math.atan(wp_x[-1],wp_y[-1])
            tx = self.Lfc*math.sin(bearing)
            ty = self.Lfc*math.cos(bearing)
        elif (wp_d >  self.Lfc).any():
            # target first point if there are no points before lookahead distance
            tx = wp_x[0]
            ty = wp_y[0]        
        
        print("x: ", tx ,", y: ", ty)
        
        return tx, ty        
    
    
    def steer_control(self, x, y):
        gamma = 2*y/(self.Lfc**2) # gamma = 1/r
        
        return gamma


    def failsafe(self, v_prev, v_current):
        if (v_current - v_prev) >= self.failsafe_thresh:
            print("Increase in velocity above maximum allowable threshold!")
            self.stop = True


# Main function
def main(args = None):
    rclpy.init(args=args)
    ppnode = PurePursuitNode()

    try:
        rclpy.spin(ppnode)
    except:
        print(Exception)
    finally:
        pass

    ppnode.stop = True
    ppnode.twist_callback()
    print("shutting down node")
    ppnode.destroy_node()
    rclpy.shutdown()
    print("done.")


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
