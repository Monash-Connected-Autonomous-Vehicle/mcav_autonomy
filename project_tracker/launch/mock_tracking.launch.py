from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    velodyne_data_dir_arg = DeclareLaunchArgument(
      'velodyne_data_dir', default_value=TextSubstitution(text="/home/home/DATASETS/KITTI/2011_09_26/2011_09_26_drive_0048_sync/velodyne_points/data/")
    )

    return LaunchDescription([
        velodyne_data_dir_arg,
        Node(
            package='project_tracker',
            executable='mock_pub.py',
            name='mock_pub',
            parameters=[
                {'velodyne_data_dir': LaunchConfiguration('velodyne_data_dir')}
            ]
        ),
        Node(
            package='project_tracker',
            executable='filter',
            name='filter',
        ),
        Node(
            package='project_tracker',
            executable='cluster.py',
            name='cluster',
        ),
    ])