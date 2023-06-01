import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from can_msgs.msg import Frame
from sensor_msgs.msg import PointCloud2 as PC2
import psutil
import time

class Supervisor(Node):

    WINDOW = 5
    IMU_FRAME_ID = 1653 # 657 in hex

    # Threshold is 80% of expected rate
    IMU_RATE_THRESHOLD = 200 * 0.8
    LIDAR_RATE_THRESHOLD = 10 * 0.8

    def __init__(self):
        super().__init__('supervisor')
        self.last_timer_reset = self.get_time().now()
        timer_period = 0.1  # seconds
        self.imu_messages = []
        self.lidar_messages = []
        self.timer = self.create_timer(timer_period, self.status_check_callback)
        self.state_publisher_ = self.create_publisher(Bool, '~/is_okay', 10)
        self.cpu_publisher_ = self.create_publisher(Float64, '~/cpu_usage', 10)
        self.ram_publisher_ = self.create_publisher(Float64, '~/ram_usage', 10)
        self.disk_publisher_ = self.create_publisher(Float64, '~/disk_usage', 10)

        self.declare_parameter('required_nodes')

        # Sensor msg rate monitor 
        self.lidar_counter, self.imu_counter = 0, 0
        self.sensor_conn_publisher_ = self.create_publisher(SensorConnection, '~/sensors_connected', 10)
        self.lidar_subscriber_ = self.create_subscription(PC2, '/velodyne_points', self.sensor_sub_callback)
        self.imu_subscriber_ = self.create_subscription(Frame, '/socketcan_reciever', self.sensor_sub_callback)
        

    def status_check_callback(self):
        current_nodes = self.get_node_names()
        required_nodes = self.get_parameter('required_nodes').get_parameter_value().string_array_value

        required_nodes_alive = check_nodes(required_nodes, current_nodes)
        self.get_logger().info(f'node status: {required_nodes_alive}')

        status_msg = Bool()
        status_msg.data = all(required_nodes_alive.values())
        self.state_publisher_.publish(status_msg)

        cpu_msg = Float64()
        cpu_msg.data = psutil.cpu_percent()
        self.cpu_publisher_.publish(cpu_msg)

        ram_msg = Float64()
        ram_msg.data = psutil.virtual_memory().percent
        self.ram_publisher_.publish(ram_msg)

        disk_msg = Float64()
        disk_msg.data = psutil.disk_usage('/').percent
        self.disk_publisher_.publish(disk_msg)

        connection_msg = SensorConnection()
        connection_msg = self.check_sensor_connection(connection_msg)
        self.sensor_conn_publisher_.publish(connection_msg)
        

    def check_sensor_connection(self, conn_msg):
        time_since_reset = self.get_time().now() - self.last_timer_reset

        # rates indicate the average number of msgs since last reset (every WINDOW amt of time)
        lidar_rate = self.lidar_counter / time_since_reset
        imu_rate = self.imu_counter / time_since_reset
        conn_msg.data.lidar_rate = imu_rate
        conn_msg.data.imu_rate = imu_rate
        conn_msg.data.imu_connected = lidar_rate > self.IMU_RATE_THRESHOLD
        conn_msg.data.lidar_connected = imu_rate > self.LIDAR_RATE_THRESHOLD

        if time_since_reset >= 5:
            self.lidar_counter, self.imu_counter = 0, 0
            self.last_timer_reset = self.get_time().now()


    def sensor_sub_callback(self, msg):
        if type(msg) == PC2:
            self.lidar_counter += 1
        
        elif msg.id == self.IMU_FRAME_ID:
            self.lidar_counter += 1





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
