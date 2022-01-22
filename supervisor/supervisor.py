import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor')
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.state_publisher_ = self.create_publisher(Bool, 'is_okay', 10)

        self.declare_parameter('required_nodes')

    def timer_callback(self):
        current_nodes = self.get_node_names()
        required_nodes = self.get_parameter('required_nodes').get_parameter_value().string_array_value

        required_nodes_alive = check_nodes(required_nodes, current_nodes)
        self.get_logger().info(f'node status: {required_nodes_alive}')

        status_msg = Bool()
        status_msg.data = all(required_nodes_alive.values())
        self.state_publisher_.publish(status_msg)


def check_nodes(required_nodes: list, current_nodes: list):
    """ For each node in required_nodes, checks if it belongs to current_nodes. 
    Args: 
        required_nodes (list[str]): Names of nodes to check whether they are alive
        current_nodes (list[str]): Nodes that are currently alive

    Returns:
        A dictionary where the keys are node names and the values are a boolean (True
        for alive)
    """
    node_status = {}
    for req_node in required_nodes:
        node_status[req_node] = req_node in current_nodes # True if alive

    return node_status


def main(args=None):
    rclpy.init(args=args)

    supervisor = Supervisor()

    rclpy.spin(supervisor)

    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
