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
        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.callback, 10)

        self.publisher1 = self.create_publisher(Range, '/ultrasonic_data1', 1)
        self.publisher2 = self.create_publisher(Range, '/ultrasonic_data2', 1)
        self.publisher3 = self.create_publisher(Range, '/ultrasonic_data3', 1)
        self.publisher4 = self.create_publisher(Range, '/ultrasonic_data4', 1)
        self.publisher5 = self.create_publisher(Range, '/ultrasonic_data5', 1)
        self.publisher6 = self.create_publisher(Range, '/ultrasonic_data6', 1)
        self.publisher7 = self.create_publisher(Range, '/ultrasonic_data7', 1)
        self.publisher8 = self.create_publisher(Range, '/ultrasonic_data8', 1)

        self.get_logger().info('NeobotixDriver node initialized.')
        self.print_vis = []

    def callback(self, frame):
        print("\033[H\033[J", end='')
        if frame.id == 0x402:
            data = frame.data
            if len(data) ==8:
                range_msg_1 = Range()
                range_msg_1.field_of_view = 0.1
                range_msg_1.radiation_type = Range.ULTRASOUND
                range_msg_1.min_range = 0.15
                range_msg_1.range = int(data[2]) / 100.0
                self.publisher1.publish(range_msg_1)   #front left
                self.print_vis.append(range_msg_1.range)

                range_msg_2 = Range()
                range_msg_2.field_of_view = 0.1
                range_msg_2.radiation_type = Range.ULTRASOUND
                range_msg_2.min_range = 0.15
                range_msg_2.range = int(data[3]) / 100.0
                self.publisher2.publish(range_msg_2)
                self.print_vis.append(range_msg_2.range)   #front center

                range_msg_3 = Range()
                range_msg_3.field_of_view = 0.1
                range_msg_3.radiation_type = Range.ULTRASOUND
                range_msg_3.min_range = 0.15
                range_msg_3.range = int(data[4]) / 100.0
                self.publisher3.publish(range_msg_3)
                self.print_vis.append(range_msg_3.range) #front right
                
                range_msg_4 = Range()
                range_msg_4.field_of_view = 0.1
                range_msg_4.radiation_type = Range.ULTRASOUND
                range_msg_4.min_range = 0.15
                range_msg_4.range = int(data[5]) / 100.0
                self.publisher4.publish(range_msg_4)
                self.print_vis.append(range_msg_4.range) #right middle
                #self.get_logger().info('frame 0x402')
                    
        if frame.id == 0x403:
            data = frame.data
            if len(data) ==8:
                range_msg_5 = Range()
                range_msg_5.field_of_view = 0.1
                range_msg_5.radiation_type = Range.ULTRASOUND
                range_msg_5.min_range = 0.15
                range_msg_5.range = int(data[2]) / 100.0
                self.publisher5.publish(range_msg_5) # right back
                self.print_vis.append(range_msg_5.range)

                range_msg_6 = Range()
                range_msg_6.field_of_view = 0.1
                range_msg_6.radiation_type = Range.ULTRASOUND
                range_msg_6.min_range = 0.15
                range_msg_6.range = int(data[3]) / 100.0
                self.publisher6.publish(range_msg_6)  #back center
                self.print_vis.append(range_msg_6.range)

                range_msg_7 = Range()
                range_msg_7.field_of_view = 0.1
                range_msg_7.radiation_type = Range.ULTRASOUND
                range_msg_7.min_range = 0.15
                range_msg_7.range = int(data[4]) / 100.0
                self.publisher7.publish(range_msg_7) #left back
                self.print_vis.append(range_msg_7.range)
                
                range_msg_8 = Range()
                range_msg_8.field_of_view = 0.1
                range_msg_8.radiation_type = Range.ULTRASOUND
                range_msg_8.min_range = 0.15
                range_msg_8.range = int(data[5]) / 100.0
                self.publisher8.publish(range_msg_8) #left middle
                self.print_vis.append(range_msg_8.range)
                #self.get_logger().info('frame 0x403')

          
            # Clearing the Screen
        if len(self.print_vis)==8:
            self.get_logger().info(
                f'\n\n'
                f'Ultrasonic sensors distance'
                f'\n\n'
                f'{self.print_vis[0]:.2f}               {self.print_vis[1]:.2f}                    {self.print_vis[2]:.2f}'
                f'\n\n'
                f'{self.print_vis[7]:.2f}                                       {self.print_vis[3]:.2f}'
                f'\n\n'
                f'{self.print_vis[6]:.2f}               {self.print_vis[5]:.2f}                    {self.print_vis[4]:.2f}'
                f'\n\n'
            )
            self.get_logger().info('\n' * 50)
            self.print_vis.clear()
            

def main():
    rclpy.init()
    neobotix_driver = NeobotixDriver()
    rclpy.spin(neobotix_driver)
    neobotix_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
