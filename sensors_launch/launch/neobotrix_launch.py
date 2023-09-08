from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_launch',
            executable='acquire_sensor',
            name='acquire_sensor',
            output='screen'
        ),
    
        Node(
            package='socketcan', 
            executable='socketcan_receiver',
            name='socketcan_receiver_node',
            output='screen', 
        )
    ])