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

    def callback(self, frame):
        if frame.id in [0x402]:
            data = frame.data
            indices = [2]  # Indices of ultrasonic data in the frame
            if len(data) == 8:
                for i, index in enumerate(indices):
                    range_msg = Range()
                    range_msg.field_of_view = 0.1
                    range_msg.radiation_type = Range.ULTRASOUND
                    range_msg.min_range = 0.15
                    range_msg.range = int(data[index]) / 100.0
                    self.publisher.publish(range_msg)
                    self.get_logger().info(f'Publishing: "{range_msg.range}"')
                    

def main():
    rclpy.init()
    neobotix_driver = NeobotixDriver()
    rclpy.spin(neobotix_driver)
    neobotix_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
