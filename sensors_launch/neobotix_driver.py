#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import Range
from can_msgs.msg import Frame
from rclpy.node import Node
#Publish deez nutz --------------- bruh
class neobotixPublisher(Node):
    def __init__(self):
        super().__init__('neobotix_driver_publisher')
        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.neobotix_callback, 10)
        
        # not too sure about the publisher - need to have different publishers for each sensors
        self.publisher_ultrasonic1 = self.create_publisher(Range,'/ultrasonic_1',1)
        self.publisher_ultrasonic2 = self.create_publisher(Range,'/ultrasonic_2',1)
        self.publisher_ultrasonic3 = self.create_publisher(Range,'/ultrasonic_3',1)
        self.publisher_ultrasonic4 = self.create_publisher(Range,'/ultrasonic_4',1)
        self.publisher_ultrasonic5 = self.create_publisher(Range,'/ultrasonic_5',1)
        self.publisher_ultrasonic6 = self.create_publisher(Range,'/ultrasonic_6',1)
        self.publisher_ultrasonic7 = self.create_publisher(Range,'/ultrasonic_7',1)
        self.publisher_ultrasonic8 = self.create_publisher(Range,'/ultrasonic_8',1)

        time_period = 0.5
        self.create_timer(time_period, self.neobotix_callback)

    """ **8 ultrasonic sensors**
        sensor 1: Front left
        sensor 2: Front center
        sensor 3: Front Right (does not work- values stays at 255)
        sensor 4: Right middle
        sensor 5: Right back
        sensor 6: Back center
        sensor 7: Left back
        sensor 8: Left Middle
        Frame: 0x402
            #sensor 4: data[5]
            #sensor 3: data[4]
            #sensor 2: data[3]
            #sensor 1: data[2]
        Frame: 0x403
            #sensor 8: data[5]
            #sensor 7: data[4]
            #sensor 6: data[3]
            #sensor 5: data[2]
    """

    def neobotix_callback(self,Frame):
        if Frame.id == 0x402:
            # Extract the sensor distance data from the frame
            range_msg_sensor4 = Range()
            range_msg_sensor3 = Range()
            range_msg_sensor2 = Range()
            range_msg_sensor1 = Range()

            range_msg_sensor1.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor1.radiation_type = Range.ULTRASOUND
            range_msg_sensor1.min_range = 0.15
            range_msg_sensor1.range = int(Frame.data[2]) / 100.0
            self.publisher_ultrasonic1(range_msg_sensor1) # Create a new Range message and publish it
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor1.range)

            range_msg_sensor2.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor2.radiation_type = Range.ULTRASOUND
            range_msg_sensor2.min_range = 0.15
            range_msg_sensor2.range = int(Frame.data[3]) / 100.0
            self.publisher_ultrasonic2(range_msg_sensor2)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor2.range)

            range_msg_sensor3.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor3.radiation_type = Range.ULTRASOUND
            range_msg_sensor3.min_range = 0.15
            range_msg_sensor3.range = int(Frame.data[4]) / 100.0
            self.publisher_ultrasonic3(range_msg_sensor3)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor3.range)

            range_msg_sensor4.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor4.radiation_type = Range.ULTRASOUND
            range_msg_sensor4.min_range = 0.15
            range_msg_sensor4.range = int(Frame.data[5]) / 100.0 
            self.publisher_ultrasonic4(range_msg_sensor4)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor4.range)

        elif Frame.id == 0x403:
            # Extract the sensor distance data from the frame
            range_msg_sensor8 = Range()
            range_msg_sensor7 = Range()
            range_msg_sensor6 = Range()
            range_msg_sensor5 = Range()

            range_msg_sensor5.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor5.radiation_type = Range.ULTRASOUND
            range_msg_sensor5.min_range = 0.15
            range_msg_sensor5.range = int(Frame.data[2]) / 100.0
            self.publisher_ultrasonic5(range_msg_sensor5)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor5.range)

            range_msg_sensor6.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor6.radiation_type = Range.ULTRASOUND
            range_msg_sensor6.min_range = 0.15
            range_msg_sensor6.range = int(Frame.data[3]) / 100.0
            self.publisher_ultrasonic6(range_msg_sensor6)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor6.range)

            range_msg_sensor7.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor7.radiation_type = Range.ULTRASOUND
            range_msg_sensor7.min_range = 0.15
            range_msg_sensor7.range = int(Frame.data[4]) / 100.0
            self.publisher_ultrasonic7(range_msg_sensor7)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor7.range)

            range_msg_sensor8.field_of_view = 0.1 # Assume a field of view of 0.1 radians==5deg => 5*2=10deg FOV x-axisoof sensor
            range_msg_sensor8.radiation_type = Range.ULTRASOUND
            range_msg_sensor8.min_range = 0.15
            range_msg_sensor8.range = int(Frame.data[5]) / 100.0
            self.publisher_ultrasonic8(range_msg_sensor8)
            self.get_logger().info('Publishing: "%s"' % range_msg_sensor8.range)
             
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



