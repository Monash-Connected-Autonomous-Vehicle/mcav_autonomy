from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_file_launch_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=TextSubstitution(text='/home/mcav/sheng_ws/control_ws/town01_path.csv'), 
        description="Absolute path of a csv file containing waypoints with columns x, y, z, yaw velocity as floats"
    )

    return LaunchDescription([
        waypoint_file_launch_arg,
        Node(
            package='velocity_planner',
            executable='velocity_planner',
            name='velocity_planner',
            parameters=[{
                'max_acceleration': 2.0,
                'local_plan_max_length': 25,
            }]
        ),
        Node(
            package='pure_pursuit',
            executable='purepursuit',
            name='purepursuit',
            parameters=[{
                'lookahead_distance': 10,
            }]
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'), # absolute path
            }]
        ),
        Node(
            package='velocity_planner',
            executable='waypoint_visualiser',
            name='waypoint_visualiser',
        ),
    ])
