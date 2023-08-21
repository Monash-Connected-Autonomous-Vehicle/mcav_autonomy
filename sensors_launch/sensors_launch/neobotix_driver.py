#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Range
from can_msgs.msg import Frame
from rclpy.node import Node
import time

class NeobotixDriver(Node):
    def __init__(self):
        super().__init__('neobotix_driver')
        # Create a subscription to the /from_can_bus topic
        self.subscription = self.create_subscription(
            Frame, '/from_can_bus', self.callback, 10)
        self.publisher = self.create_publisher(Range, '/ultrasonic_data', 1)
        self.get_logger().info('NeobotixDriver node initialized.')

    def callback(self, frame):
        if frame.id in [0x402,0x403]:
            data = frame.data
            indices = [2,3,4,5]  # Indices of ultrasonic data in the frame
            sensor_names = [
                "Front left", "Front center", "Front Right", "Right middle",
                "Right back", "Back center", "Left back", "Left Middle"
            ]
            if len(data) == 8:
                for i, index in enumerate(indices):
                    range_msg = Range()
                    range_msg.field_of_view = 0.1
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.min_range = 0.15
                    range_msg.range = int(data[index]) / 100.0
                    sensor_name = sensor_names[i]
                    self.publisher.publish(range_msg)
                    self.get_logger().info(f'{sensor_name}: {range_msg.range}m')
                self.get_logger().info('------')
                time.sleep(0.1)   
                    

def main():
    rclpy.init()
    neobotix_driver = NeobotixDriver()
    rclpy.spin(neobotix_driver)
    neobotix_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
#!/usr/bin/env python3
import rclpy
import time
from collections import deque
from sensor_msgs.msg import Range
from can_msgs.msg import Frame
from rclpy.node import Node

class NeobotixDriver(Node):
    def __init__(self):
        super().__init__('neobotix_driver')
        self.subscription = self.create_subscription(
            Frame, '/from_can_bus', self.callback, 10)
        self.publisher = self.create_publisher(Range, '/ultrasonic_data', 1)
        self.get_logger().info('NeobotixDriver node initialized.')
        
        self.sensor_buffer = [deque(maxlen=5) for _ in range(8)]  # Create a buffer for each sensor

    def apply_moving_average(self, sensor_index, new_value):
        self.sensor_buffer[sensor_index].append(new_value)
        return sum(self.sensor_buffer[sensor_index]) / len(self.sensor_buffer[sensor_index])

    def callback(self, frame):
        if frame.id in [0x402, 0x403]:
            data = frame.data
            indices = [2, 3, 4, 5]  # Indices of ultrasonic data in the frame
            sensor_names = [
                "Front left", "Front center", "Front Right", "Right middle",
                "Right back", "Back center", "Left back", "Left Middle"
            ]
            if len(data) == 8:
                for i, index in enumerate(indices):
                    range_msg = Range()
                    range_msg.field_of_view = 0.1
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.min_range = 0.15
                    raw_value = int(data[index]) / 100.0
                    filtered_value = self.apply_moving_average(i, raw_value)
                    sensor_name = sensor_names[i]
                    self.publisher.publish(range_msg)
                    self.get_logger().info(f'{sensor_name}: {filtered_value:.2f}')
                self.get_logger().info('----')
                time.sleep(0.1)  # Shorter delay for better real-time representation

def main():
    rclpy.init()
    neobotix_driver = NeobotixDriver()
    rclpy.spin(neobotix_driver)
    neobotix_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""