#!/usr/bin/env python3
"""
This node is a publisher that sends a Can Frame to the /to_can_bus
topic, it sends the message ever 0.5s continously until the script
is closed. The Frame is message sent to the write imu sensor (657h)
setting it's X, Y and Z acceleration.
"""

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame

class CalibrationPublisher(Node):
    """
    Publisher node that sends CAN Frame to IMU senesor

    :CAN Frame:
        FRAME ID: the WRITE to IMU frame id is 657 HEX however,
                  CAN recieves decimal so its sent as 1623
        DLC: data length, write imu data only needs 4 bytes (X, Y, Z, Start flag)
        Data: Frame recieves 8 bytes so data recieves trailing 0s. First 4 bytes indicat:
            - X: x-acceleration (in Gs)
            - Y: y-acceleration (in Gs)
            - Z: z-acceleration - vertical plane (in Gs)
            - Start Calib: flag to enable message
        
        X, Y and Z default to 0, 0 and 1 but to set acceleration follows:
            - 0: 0G
            - 1: +1G
            - 2: -1G
        
    """
    FRAME_ID = 1623 
    
    def __init__(self, x=0, y=0, z=1):
        super().__init__('calibration_publisher')
        self.publisher_ = self.create_publisher(Frame, '/to_can_bus', 1)
        self.data = [x, y, z, 1, 0, 0, 0, 0]
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
