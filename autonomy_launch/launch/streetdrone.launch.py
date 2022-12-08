from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoint_filename', default_value=TextSubstitution(text='/home/mcav/mcav_ws/src/mcav_autonomy/N1Level4_rectangularPath_Waypoints.csv'),
        description="Absolute path of a csv file containing waypoints with columns x, y, z, yaw velocity as floats"
    )

    return LaunchDescription([
        # Arguments
        waypoint_file_launch_arg,
        # Sensing
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('mcav_sd_sensing'), 'launch', 'velodyne.launch.xml')
            ]),
        ),
        # Perception
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('project_tracker'), 'launch', 'tracking.launch.xml')
            ]),
        ),
        # Planning
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('velocity_planner'), 'launch', 'velocity_planner.launch.py')
            ]),
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoint_filename': LaunchConfiguration('waypoint_filename'), # absolute path
            }]
        ),
        # Control
        Node(
            package='pure_pursuit',
            executable='purepursuit',
            name='purepursuit',
        ),
        # Visualisation
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('autonomy_launch'), 'autonomy.rviz')]
        )
    ])


