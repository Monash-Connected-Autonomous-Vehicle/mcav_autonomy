from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_tracker',
            executable='mock_pub',
            name='mock_pub',
        ),
        Node(
            package='pcl_bind',
            executable='filter',
            name='filter',
        ),
        Node(
            package='project_tracker',
            executable='cluster',
            name='cluster',
        ),
    ])