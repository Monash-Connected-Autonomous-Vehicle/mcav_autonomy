#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Range
from can_msgs.msg import Frame
from rclpy.node import Node

class neobotixPublisher(Node):
    def __init__(self):
        super().__init__('neobotix_driver_publisher')
        self.subscription = self.create_subscription(
            Frame, '/from_can_bus', self.neobotix_callback, 10)
        self.publisher = self.create_publisher(Frame,'/to_can_bus',1)

        time_period = 0.5
        self.create_timer(time_period, self.neobotix_callback)

    def neobotix_callback(self,Frame):
        if Frame.id == 0x402 or Frame.id == 0x403:
            # Extract the sensor ID and distance data from the frame
            sensor_id = Frame.id - 0x400
            distance = Frame.data[0] / 100.0
            # Create a new Range message and publish it
            range_msg = Range()
            range_msg.header.frame_id = f"ultrasonic_{sensor_id}"
            range_msg.field_of_view = 0.1  # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.min_range = 0.15  # Minimum detectable 5cm
            range_msg.range = distance
            self.publisher.publish(range_msg)
      
# Once driver is ready
# ultrasonic message that encodes the range info for all the sensors
def main():
    rclpy.init()
    neobotix_driver = neobotixPublisher()
    rclpy.spin(neobotix_driver)
    neobotix_driver.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()




