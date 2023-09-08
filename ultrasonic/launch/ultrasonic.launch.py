from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():
    socket_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace=TextSubstitution(text=''),
        output='screen')
    
    return LaunchDescription([
        socket_can_receiver_node,
        Node(
            package='ultrasonic',
            executable='reader',
            name='ultrasonic'
        ),
    ])