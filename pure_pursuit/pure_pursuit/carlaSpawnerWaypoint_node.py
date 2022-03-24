#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
import carla
from carla import Transform, Location, Rotation, World
from mcav_interfaces.msg import WaypointArray, DetectedObjectArray, Waypoint


class WaypointPublisher(Node):


    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoints_pub = self.create_publisher(WaypointArray, 'vel_waypoints', 10)
        timer_period = 0.5  # seconds
        self.spinner = self.create_timer(timer_period, self.waypoints_callback)
        
        # waypoint generation arguments
        self.debug = None
        self.map = None
        self.vehicle = None
        self.vehicle_pos = None
        self.path = None
        self.length_path = 50 # variable
        self.length_seg = 10  # variable; must be shorter than length_path
        self.target_vel = 2.0 # target velocity (m/s)


    # Main callback function for publishing waypoints
    def waypoints_callback(self):
        self.vehicle_pos = self.vehicle.get_transform() # record vehicle pose on each loop
        wp_list_msg = WaypointArray()
        path_seg = self.waypoints_seg() # get a segment of the waypoint path
        wp_list_msg.waypoints = self.waypoints2msg(path_seg) # converts segment to WaypointArray message
        self.draw_waypoints(self.path, carla.Color(0, 0, 255)) # highlight waypoint path blue
        self.draw_waypoints(path_seg, carla.Color(255, 0, 0)) # highlight segment red
        (self.waypoints_pub).publish(wp_list_msg) 


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
              
                
    # Gets waypoint segment from waypoint path
    def waypoints_seg(self):
        # start point, s, is the point closest to the vehicle
        wp_x = np.array([wp.transform.location.x - self.vehicle_pos.location.x for wp in self.path])
        wp_y = np.array([wp.transform.location.y - self.vehicle_pos.location.y for wp in self.path])
        wp_d = np.hypot(wp_x, wp_y)
        s = np.argmin(wp_d)

        e = s + self.length_seg
        path_seg = self.path[s:e]
        
        return path_seg
    
    
    # Converts waypoint to Waypoint message
    def waypoints2msg(self, waypoints):
        wp_msg_list = [None]*len(waypoints)
        
        for i in range(len(wp_msg_list)):
            wp_msg = Waypoint()
            
            v_yaw = self.vehicle_pos.rotation.yaw
            v_x = self.vehicle_pos.location.x
            v_y = self.vehicle_pos.location.y
            wp_x = waypoints[i].transform.location.x
            wp_y = waypoints[i].transform.location.y
            wp_z = waypoints[i].transform.location.z
            
            # transform waypoint coordinates to vehicle frame
            rad_factor = math.pi/180
            wp_msg.pose.position.x = ((wp_x-v_x)*math.cos(-v_yaw*rad_factor)
                                     -(wp_y-v_y)*math.sin(-v_yaw*rad_factor))
            wp_msg.pose.position.y = ((wp_x-v_x)*math.sin(-v_yaw*rad_factor)
                                     +(wp_y-v_y)*math.cos(-v_yaw*rad_factor))
            wp_msg.pose.position.z = wp_z
            
            # waypoint velocity is zero upon reaching last waypoint
            if i == len(wp_msg_list)-1:
                wp_msg.velocity.linear.x = 0.0
            else:
                wp_msg.velocity.linear.x = self.target_vel
                
            wp_msg_list[i] = wp_msg  
       
        return wp_msg_list
        
        
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
        client.set_timeout(10.0) # seconds
        world = client.load_world("Town02")
        # client.load_world('Town02')
        # client.reload_world()

        # spawn subaru
        blueprint_library = world.get_blueprint_library()
        sub_bp = blueprint_library.filter("model3")[0]
        sub_bp.set_attribute("role_name", "ego_vehicle")
        spawn_point = Location(x=46.100533, y=236.447159, z=0.5)
        sub_tf = Transform(spawn_point, Rotation(0,-90,0))
        vehicle = world.spawn_actor(sub_bp, sub_tf)

        # enable built-in autopilot
        vehicle.set_autopilot(False)

        # keep track of actors
        actor_list = []
        actor_list.append(vehicle)
        
        # set the spectator view
        top_down_view = True
        if top_down_view:
		        spectator_transform = vehicle.get_transform()
		        spectator_transform.location += carla.Location(x = -30.0, y=-30.0, z = 50.0)
		        spectator_transform.rotation = carla.Rotation(pitch=-90, yaw=180)
        else:
            spectator_transform = vehicle.get_transform()
            spectator_transform.location += carla.Location(x = 0, y=10, z = 6)
            spectator_transform.rotation = carla.Rotation(pitch=-30, yaw=-90)
        world.get_spectator().set_transform(spectator_transform)	
		       
        #set node parameters and spin
        world = client.get_world()
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
        print("destroying actors")
        for actor in actor_list:
            actor.destroy()
        # ego_cam.destroy()
        print("done.")


if __name__ == '__main__':
    main()
