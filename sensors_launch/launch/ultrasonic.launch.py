from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_socketcan',
            executable='socket_can_receiver_node_exe',
            name='socket_can_receiver'
        ),
        Node(
            package='ultrasonic',
            executable='reader',
            name='ultrasonic'
        ),
    ])