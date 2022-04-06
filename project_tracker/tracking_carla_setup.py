import carla
from carla import Transform, Location, Rotation, World

def attach_camera(actor, world):
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_location = carla.Location(0, 0, 1.2)
    cam_rotation = carla.Rotation(0, 0, 0)
    cam_transform = carla.Transform(cam_location,cam_rotation)
    world.spawn_actor(cam_bp, cam_transform, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)

def attach_lidar(actor, world):
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    lidar_location = carla.Location(0, 0, 2.4)
    lidar_rotation = carla.Rotation(0, 0, 0)
    lidar_bp.set_attribute("channels", "16")
    lidar_bp.set_attribute("lower_fov", "-15")
    lidar_bp.set_attribute("upper_fov", "15")
    lidar_bp.set_attribute("range", "100")
    lidar_bp.set_attribute("points_per_second", "300000")
    lidar_bp.set_attribute("dropoff_general_rate", "0.10")
    lidar_transform = carla.Transform(lidar_location, lidar_rotation)
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=actor, attachment_type=carla.AttachmentType.Rigid)

def setup_tracking_scenario():
    try:
        # Create a client to communicate with the server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.load_world("Town04")

        # Spawn streetdrone
        blueprint_library = world.get_blueprint_library()
        sdrone_bp = blueprint_library.filter("cooper_s_2021")[0]
        sdrone_bp.set_attribute("role_name", "hero")
        spawn_point = world.get_map().get_spawn_points()[0]
        sdrone = world.spawn_actor(sdrone_bp, spawn_point)

        # Attach sensors
        attach_camera(sdrone, world)
        attach_lidar(sdrone, world)

        # disable built-in autopilot
        sdrone.set_autopilot(False)

        # store list of actors
        actors = [sdrone]

        while True:
            spectator_transform = sdrone.get_transform()
            spectator_transform.location += carla.Location(x = 0, y = 10, z = 6)
            spectator_transform.rotation  = carla.Rotation(pitch = -30, yaw=-90)
            world.get_spectator().set_transform(spectator_transform)
    except KeyboardInterrupt:
        pass
    finally:
        print("destroying all spawned actors")
        for actor in actors:
            actor.destroy()


if __name__ == '__main__':
    setup_tracking_scenario()