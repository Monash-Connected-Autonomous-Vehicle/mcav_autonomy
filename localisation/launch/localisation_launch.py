"""Use the lidar and a reference map to perform global registatration and local refinement to find the initial pose for navigation"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localisation',
            executable='rough_pose_guess',
            name='global_registration',
            remappings=[
                ('/pointcloud_map', '/local_map'),
                ('/lidar_points', '/velodyne_points'),
            ]
        ),
        Node(
            package='localisation',
            executable='pose_initialisation',
            name='local_refinement',
            remappings=[
                ('/pointcloud_map', '/local_map'),
                ('/lidar_points', '/velodyne_points'),
            ]
        ),
    ])
