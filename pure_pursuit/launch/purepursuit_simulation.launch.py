from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=TextSubstitution(text='/home/mcav/sheng_ws/control_ws/town01_path.csv'), 
        description="Absolute path of a csv file containing waypoints with columns x, y, z, yaw velocity as floats"
    )
    
    return LaunchDescription([
        waypoint_file_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('pure_pursuit'),
                '/purepursuit.launch.py']),
            launch_arguments={
                "waypoints_file": LaunchConfiguration('waypoints_file'),
            }.items()
        ),
        Node(
            package='pure_pursuit',
            executable='carla_global_planner',
            name='carla_global_planner',
        ),
    ])
