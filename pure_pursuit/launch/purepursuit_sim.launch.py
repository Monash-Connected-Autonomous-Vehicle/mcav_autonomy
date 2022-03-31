from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='carlaSpawner_node',
            name='carla_spawner',
        ),
        Node(
            package='pure_pursuit',
            executable='carlaGlobalPathGenerator_node',
            name='carla_global_path_generator',
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
            executable='ppsimple_node',
            name='simple_pure_pursuit',
        ),
    ])
