from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoint_filename', default_value=TextSubstitution(text='/home/mcav/Sheng/control_ws/town01_path.csv')
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('pure_pursuit'),
                '/purepursuit_simulation.launch.py']),
            launch_arguments={
                "waypoint_filename": LaunchConfiguration(waypoint_file_launch_arg),
            }
        ),
        Node(
            package='pure_pursuit',
            executable='carla_spawner',
            name='carla_spawner',
        ),
    ])
