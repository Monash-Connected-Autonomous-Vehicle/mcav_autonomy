from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoint_filename', default_value=TextSubstitution(text='/home/mcav/sheng_ws/control_ws/town01_path.csv'), 
        description="Absolute path of a csv file containing waypoints with columns x, y, z, yaw velocity as floats"
    )

    return LaunchDescription([
        waypoint_file_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('pure_pursuit'),
                '/purepursuit_simulation.launch.py']),
            launch_arguments={
                "waypoint_filename": LaunchConfiguration('waypoint_filename'),
            }.items()
        ),
        Node(
            package='pure_pursuit',
            executable='carla_spawner',
            name='carla_spawner',
        ),
    ])
