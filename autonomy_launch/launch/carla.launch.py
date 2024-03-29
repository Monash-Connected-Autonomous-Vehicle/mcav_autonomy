from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=TextSubstitution(text='/home/mcav/Sheng/control_ws/town01_path.csv'), 
        description="Absolute path of a csv file containing waypoints with columns x, y, z, yaw, velocity as floats"
    )

    return LaunchDescription([
        # Arguments
        waypoint_file_launch_arg,
        # Carla setup
        Node(
            package='simulation',
            executable='carla_spawn',
            name='carla_spawn',
        ),
        Node(
            package='simulation',
            executable='generate_traffic',
            name='generate_traffic',
        ),
        Node(
            package='pure_pursuit',
            executable='carla_global_planner',
            name='carla_global_planner',
            remappings=[
                ('/twist_cmd', '/carla/ego_vehicle/twist'),
            ]
        ),
        Node(
            package='carla_twist_to_control',
            executable='carla_twist_to_control',
            name='carla_twist_to_control',
        ),
        # Autonomy stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('autonomy_launch'),
                '/autonomy.launch.py']),
            launch_arguments={
                "waypoints_file": LaunchConfiguration('waypoints_file'),
            }.items()
        ),
    ])

