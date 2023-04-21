#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame

class CalibrationPublisher(Node):
    
    FRAME_ID = 1623 # 657 in hexadecimal
    
    def __init__(self, x=0, y=0, z=1):
        super().__init__('calibration_publisher')
        self.publisher_ = self.create_publisher(Frame, '/to_can_bus', 1)
        self.data = [x, y, z, 0, 0, 0, 0]
        # self.data = [2 if i == -1 else i for i in data] 
        
        time_period = 0.5
        self.create_timer(time_period, self.send_message)
    
    def send_message(self):
        msg = Frame()
        msg.dlc = 4
        msg.id = self.FRAME_ID
        msg.data = self.data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: ID: {} Data: {}'.format(self.FRAME_ID, msg.data))


def main():
    rclpy.init()
    calibrator = CalibrationPublisher()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
