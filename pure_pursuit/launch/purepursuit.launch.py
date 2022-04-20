from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoint_filename', default_value=TextSubstitution(text='/home/mcav/Sheng/control_ws/town01_path.csv')
    )

    return LaunchDescription([
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
            executable='purepursuit',
            name='purepursuit',
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoint_filename': LaunchConfiguration(waypoint_file_launch_arg), # absolute path
            }]
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_visualiser',
            name='waypoint_visualiser',
        ),
    ])
