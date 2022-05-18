#!/usr/bin/env python3
import math
import time
import numpy as np
import rclpy
import carla
import csv
from rclpy.node import Node
from carla import Transform, Location, Rotation, World
from mcav_interfaces.msg import WaypointArray, Waypoint
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from typing import Tuple, List


total_waypoints = 210
waypoint_distance = 2 # distance between waypoints (m)
target_vel = 4.3 # target velocity (m/s)


class WaypointPublisher(Node):
    """
    Node Class - generates and visalises waypoints and lookahead distance in carla; publishes pose and global waypoint (optional) information.
    """

    def __init__(self):
        super().__init__('carla_global_planner')
        # subscribers
        self.local_wp_sub = self.create_subscription(WaypointArray, 'local_map_waypoints', self.show_local_wp, 10)
        self.local_wp_sub  # prevent 'unused variable' warning
        self.global_wp_sub = self.create_subscription(WaypointArray, 'global_waypoints', self.show_global_wp, 10)
        self.global_wp_sub  # prevent 'unused variable' warning
        self.target_sub = self.create_subscription(PoseWithCovarianceStamped, 'target_pose', self.show_target, 10)
        
        # publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'current_pose', 1)
        self.waypoints_pub = self.create_publisher(WaypointArray, 'global_waypoints', 10)
        timer_period = 0.5  # seconds
        self.spinner = self.create_timer(timer_period, self.waypoints_callback)
        
        # function variables
        self.debug = None
        self.map = None
        self.vehicle = None
        self.vehicle_pos = None
        self.path = None


    def waypoints_callback(self):
        """
        Main callback function for publishing waypoints.
        """
        self.vehicle_pos = self.vehicle.get_transform() # record vehicle pose on each loop 
        pose_msg = self.pose2msg()
        (self.pose_pub).publish(pose_msg)
        
        # TODO: comment this section out if using other means of publishing global waypoints
        # wp_list_msg = WaypointArray()
        # wp_list_msg.waypoints = self.waypoints2msg(self.path) # converts to WaypointArray message
        # (self.waypoints_pub).publish(wp_list_msg)


    def gen_waypoints(self) -> List[carla.Waypoint]:    
        """Generates waypoint path in CARLA.

        Returns:
            List[carla.Waypoint]: list of carla waypoints
        """

        wp_start = (self.map).get_waypoint(self.vehicle.get_transform().location) # generates a waypoint at vehicle start location
        wp_next = wp_start
        wp_list = [None]*(total_waypoints)
        
        for i in range(len(wp_list)):
            wp_list[i] = wp_next
            # chooses the first waypoint in a list of waypoints that are a distance away
            wp = wp_next.next(waypoint_distance)
            wp_next = wp[-1]

        return wp_list
    
    
    def pose2msg(self) -> PoseWithCovarianceStamped:
        """Converts CARLA pose to PoseWithCovarianceStamped message.

        Returns:
            PoseWithCovarianceStamped: pose message
        """        

        rad_factor = math.pi/180
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.vehicle_pos.location.x
        pose_msg.pose.pose.position.y = -self.vehicle_pos.location.y
        pose_msg.pose.pose.position.z = self.vehicle_pos.location.z 
        pose_msg.pose.pose.orientation.z = -self.vehicle_pos.rotation.yaw*rad_factor
          
        return pose_msg
    
    
    def waypoints2msg(self, waypoints: List[carla.Waypoint]) -> List[Waypoint]:
        """Converts waypoint to Waypoint message.

        Args:
            waypoints (List[carla.Waypoint]): list of CARLA waypoints

        Returns:
            List[Waypoint]: list of Waypoint messages
        """        

        rad_factor = math.pi/180
        wp_msg_list = [None]*len(waypoints)

        for i in range(len(wp_msg_list)):
            v_yaw = self.vehicle_pos.rotation.yaw
            v_x = self.vehicle_pos.location.x
            v_y = self.vehicle_pos.location.y
            wp_x = waypoints[i].transform.location.x
            wp_y = waypoints[i].transform.location.y
            wp_z = waypoints[i].transform.location.z
            wp_yaw = waypoints[i].transform.rotation.yaw
            
            # transform waypoint coordinates to vehicle frame        
            wp_msg = Waypoint()
            wp_msg.frame_id = "map"
            wp_msg.pose.position.x = wp_x
            wp_msg.pose.position.y = -wp_y
            wp_msg.pose.position.z = wp_z
            wp_msg.pose.orientation.z = -wp_yaw*rad_factor
            
            # waypoint velocity is zero upon reaching last waypoint
            if i == len(wp_msg_list)-1:
                wp_msg.velocity.linear.x = 0.0
            else:
                wp_msg.velocity.linear.x = target_vel
                
            wp_msg_list[i] = wp_msg  
          
        return wp_msg_list


    def euler_z(self, px: float, py: float, pz: float, fx: float, fy: float, fz: float, fa: float) -> np.array: 
        """Transforms coordinate of point from start frame to target frame, rotating about the z-axis.

        Args:
            px (float): point x from start frame
            py (float): point y from start frame
            pz (float): point z from start frame
            fx (float): point x from target frame
            fy (float): point y from target frame
            fz (float): point z from target frame
            fa (float): point yaw bearing from target frame in radians

        Returns:
            np.array: coordinate of point from target frame
        """

        C = math.cos(fa)
        S = math.sin(fa)

        rot_z       =    np.matrix([[C, -S, 0, 0],
                                    [S, C, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

        translation =    np.matrix([[1, 0, 0, fx], 
                                    [0, 1, 0, fy], 
                                    [0, 0, 1, fz],
                                    [0, 0, 0, 1]])

        point       =    np.matrix([[px], 
                                    [py], 
                                    [pz],
                                    [1]])

        result = np.matmul(rot_z, point)
        result = np.matmul(translation, result)

        return result


    def show_target(self, target_msg: PoseWithCovarianceStamped):
        """Visualise lookahead target in CARLA.

        Args:
            target_msg (PoseWithCovarianceStamped): target pose from vehicle frame
        """        

        if self.vehicle_pos:
            rad_factor = math.pi/180.0
            tx = target_msg.pose.pose.position.x
            ty = -target_msg.pose.pose.position.y
            tz = self.vehicle_pos.location.z
            fx = self.vehicle_pos.location.x
            fy = self.vehicle_pos.location.y
            fz = self.vehicle_pos.location.z
            fa = self.vehicle_pos.rotation.yaw

            result = self.euler_z(tx, ty, tz, fx, fy, fz, fa*rad_factor)

            begin = carla.Location(x=self.vehicle_pos.location.x,        
                            y=self.vehicle_pos.location.y, 
                            z=self.vehicle_pos.location.z)
            end =   carla.Location(x=result[0,0],        
                            y=result[1,0], 
                            z=result[2,0])

            (self.debug).draw_line(begin, end, 0.1, carla.Color(0,255,0), 0.08)
            print("Lookahead point: ", end)

   
    def show_local_wp(self, wp_msg_list: WaypointArray):
        """Visualise local waypoints in CARLA. 

        Args:
            wp_msg_list (WaypointArray): array of waypoints of type Waypoint
        """        

        self.draw_waypoints(wp_msg_list, carla.Color(255, 0, 0)) 


    def show_global_wp(self, wp_msg_list: WaypointArray):
        """Visualise global waypoints in CARLA.

        Args:
            wp_msg_list (WaypointArray): array of waypoints of type Waypoint
        """

        self.draw_waypoints(wp_msg_list, carla.Color(0, 0, 255)) 
        
           
    def draw_waypoints(self, wp_msg_list: WaypointArray, color: int):
        """Visualises waypoint paths.

        Args:
            wp_msg_list (WaypointArray): array of waypoints of type Waypoint
            color (int): carla.Color(R,G,B)
        """

        wp_list = []
        for wp_msg in wp_msg_list.waypoints:
            wp_location = carla.Location(x=wp_msg.pose.position.x,        
                                         y=-wp_msg.pose.position.y, 
                                         z=wp_msg.pose.position.z)
            wp_list.append(wp_location)
    
        for i in wp_list:
            (self.debug).draw_point(i + carla.Location(z=0.25), 0.05, color, False)
            
           
    def save_path(self, waypoints: List[carla.Waypoint]):
        """Save path as .csv.

        Args:
            waypoints (List[carla.Waypoint]): list of CARLA waypoints
        """

        with open('town01_path.csv', 'w', encoding='UTF8', newline='') as f:
            fieldnames = ['x', 'y', 'z', 'yaw', 'velocity', 'change_flag']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for wp in waypoints:
                wp_x = wp.transform.location.x
                wp_y = -wp.transform.location.y
                wp_z = wp.transform.location.z
                wp_yaw = -wp.transform.rotation.yaw
                wp_v = target_vel
                writer.writerow({'x': wp_x, 
                                 'y': wp_y, 
                                 'z': wp_z, 
                                 'yaw': wp_yaw, 
                                 'velocity': wp_v, 
                                 'change_flag': 0})


def main(args=None):
    # node initialisation
    rclpy.init(args=args)
    wp_pub = WaypointPublisher()

    try:
        client = carla.Client('localhost', 2000)  # create a client to communicate with the server
        world = client.get_world()
        vehicle = None
        while vehicle == None:
            for actor in world.get_actors():
                try:
                    if actor.attributes["role_name"] == "ego_vehicle": # finds main car
                        vehicle = actor
                        break 
                    else:
                        print("Unable to find vehicle. Make sure you have spawned a vehicle with role_name 'ego_vehicle'.")
                except KeyError:
                    print("Unable to find vehicle. Make sure you have spawned a vehicle with role_name 'ego_vehicle'.")
                
        #set node parameters and spin
        wp_pub.debug = world.debug
        wp_pub.map = world.get_map()
        wp_pub.vehicle = vehicle
        wp_pub.path = wp_pub.gen_waypoints()
        wp_pub.save_path(wp_pub.path)
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
