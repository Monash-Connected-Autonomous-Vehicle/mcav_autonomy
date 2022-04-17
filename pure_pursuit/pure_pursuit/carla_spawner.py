#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
import carla
from carla import Transform, Location, Rotation, World


def main():
    world = None

    try:
        # create a client to communicate with the server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) # seconds
        world = client.load_world("Town01")
#        world = client.load_world("Town02")
#        world = client.load_world("Town10HD")

        # spawn subaru
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("model3")[0]
        vehicle_bp.set_attribute("role_name", "ego_vehicle")
#        spawn_point = Location(x=-52.0, y=30.0, z=0.5)
        # spawn_point = Location(x=46.100533, y=231.447159, z=0.5)
        spawn_point = Location(x=192, y=133, z=0.8)
        # spawn_point = Location(x=92.5, y=50, z=0.8)
        vehicle_tf = Transform(spawn_point, Rotation(0,0,0))
        vehicle = world.spawn_actor(vehicle_bp, vehicle_tf)
        print(world)
        print(vehicle)
        
        # set the spectator view
        while True:
            spectator_transform = vehicle.get_transform()
            spectator_transform.location += carla.Location(x=0.0, y=0.0, z =50.0)
            spectator_transform.rotation = carla.Rotation(pitch=-90, yaw=180)
            world.get_spectator().set_transform(spectator_transform)

    except KeyboardInterrupt:
        # https://github.com/carla-simulator/carla/issues/1346
        print("User Cancelled")
    finally:
        print("destroying actors")
        vehicle.destroy()
        # ego_cam.destroy()
        print("done.")


if __name__ == '__main__':
    main()
