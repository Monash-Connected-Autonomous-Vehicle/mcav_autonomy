from launch import LaunchDescription
from launch_ros.actions import Node

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
                'max_velocity': 3.0,
            }]
        ),
        Node(
            package='pure_pursuit',
            executable='purepursuit',
            name='purepursuit',
        ),
    ])
