# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import math
# from ros2_socketcan import socketcan_sender, socketcan_receiver

class Ultrasonic(Node):
    MIN_RANGE = 0.15 # meters
    MAX_RANGE = 5.5 # meters
    FIELD_OF_VIEW = 70 / 180 * math.pi


    def __init__(self):
        super().__init__('ultrasonic')
        self.frame1_id = 0x402
        self.frame2_id = 0x403
        self.topics = ('front_left', 'front_center', 'front_right', 'right_middle',
                       'right_back', 'back_center', 'left_back', 'left_middle')
        

        self.map_frame1 = {
            2: self.create_publisher(Range, "/front_left", 10),
            3: self.create_publisher(Range, "/front_center", 10),
            4: self.create_publisher(Range, "/front_right", 10),
            5: self.create_publisher(Range, "/right_middle", 10),
        }
        

        self.map_frame2 = {
            2: self.create_publisher(Range, "/right_back", 10),
            3: self.create_publisher(Range, "/back_center", 10),
            4: self.create_publisher(Range, "/left_back", 10),
            5: self.create_publisher(Range, "/left_middle", 10),
        }

        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.read_msg, 10)


    def read_msg(self, msg: Frame):
        # initialise the Range message
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = 0
        range_msg.max_range = self.MAX_RANGE
        range_msg.min_range = self.MIN_RANGE
        range_msg.field_of_view = self.FIELD_OF_VIEW

        if msg.id == self.frame1_id:
            for i in self.map_frame1:
                range_msg.range = msg.data[i]
                self.map_frame1[i].publish(range_msg)

        elif msg.id == self.frame2_id:
            for i in self.map_frame2:
                range_msg.range = msg.data[i]
                self.map_frame2[i].publish(range_msg)
    
def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = Ultrasonic()

    rclpy.spin(lidar_subscriber)

    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Range
# from std_msgs.msg import Header

# class RangePublisher(Node):
#     def __init__(self):
#         super().__init__('range_publisher')
#         self.publisher_ = self.create_publisher(Range, 'range_topic', 10)
#         self.timer_ = self.create_timer(1.0, self.publish_range)

#     def publish_range(self):
#         range_msg = Range()
#         range_msg.header = Header()
#         range_msg.header.stamp = self.get_clock().now().to_msg()
#         range_msg.radiation_type = 0  # Ultrasound
#         range_msg.field_of_view = 0.5  # Example field of view in radians
#         range_msg.min_range = 0.1  # Example minimum range in meters
#         range_msg.max_range = 5.0  # Example maximum range in meters
#         range_msg.range = 2.3  # Example measured range in meters

#         self.publisher_.publish(range_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RangePublisher()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
