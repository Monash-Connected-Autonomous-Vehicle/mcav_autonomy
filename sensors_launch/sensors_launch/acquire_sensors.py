#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Range
from can_msgs.msg import Frame
from rclpy.node import Node


class Neobotrix(Node):
    def __init__(self):
        super().__init__('neobotrix_driver')
        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.something, 10)
        self.subscription
        self.publisher1 = self.create_publisher(Range, '/ultrasonic_data1', 1)
        self.publisher2 = self.create_publisher(Range, '/ultrasonic_data2', 1)
        self.publisher3 = self.create_publisher(Range, '/ultrasonic_data3', 1)
        self.publisher4 = self.create_publisher(Range, '/ultrasonic_data4', 1)
        self.publisher5 = self.create_publisher(Range, '/ultrasonic_data5', 1)
        self.publisher6 = self.create_publisher(Range, '/ultrasonic_data6', 1)
        self.publisher7 = self.create_publisher(Range, '/ultrasonic_data7', 1)
        self.publisher8 = self.create_publisher(Range, '/ultrasonic_data8', 1)
        
def something(self,msg):
    if msg.id == 0x402 or msg.id == 0x403:
        self.get_logger().info(f'Acquired sensor message from CAN frame {msg.id}: {msg.data}')

def main(args=None):
    rcply.init(args=args)
    node = Neobotrix()
    rcply.spin(node)
    node.destroy_node()
    rcply.shutdown()

if __name__ == '__main__':
    main()