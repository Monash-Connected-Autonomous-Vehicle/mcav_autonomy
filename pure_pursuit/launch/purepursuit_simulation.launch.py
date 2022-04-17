from launch import LaunchDescription
from launch_ros.actions import Node

waypoint_filename = input("Waypoint directory: ")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='carla_spawner',
            name='carla_spawner',
        ),
        Node(
            package='pure_pursuit',
            executable='carla_global_planner',
            name='carla_global_planner',
        ),
        Node(
            package='velocity_planner',
            executable='velocity_planner',
            name='velocity_planner',
            parameters=[{
                'max_acceleration': 0.5,
                'local_plan_max_length': 25,
            }]
        ),
        Node(
            package='pure_pursuit',
            executable='purepursuit',
            name='purepursuit',
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoint_filename': waypoint_filename,
            }]
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_visualiser',
            name='waypoint_visualiser',
        ),
    ])
