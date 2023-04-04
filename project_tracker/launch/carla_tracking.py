from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_tracker',
            executable='filter',
            name='filter',
            remappings=[('/velodyne_points', '/carla/ego_vehicle/lidar'),]
        ),
        Node(
            package='project_tracker',
            executable='cluster.py',
            name='cluster',
        ),
    ])
