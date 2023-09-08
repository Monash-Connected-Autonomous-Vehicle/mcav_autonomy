from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='socketcan',
            namespace='socketcan_receiver',
            executable='socketcan_receiver',
            name='socketcan_receiver'
        ),
        Node(
            package='sensors_launch',
            namespace='neobotix_driver',
            executable='neobotix_driver',
            name='neobotix_driver'
        ),
    ])