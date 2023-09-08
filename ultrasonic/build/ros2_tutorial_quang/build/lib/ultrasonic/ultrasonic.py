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
from typing import NamedTuple
from can_msgs import Frame


class Ultrasonic(Node):

    def __init__(self):
        super().__init__('ultrasonic')
        self.is_active = False
        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.read_msg, 10)
        # self.publisher_ = self.create_publisher(Bool, '/ebrake_is_active', 10)
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish)

    def read_msg(self, msg):
        
        self.get_logger().info(f"{msg.id}, {msg.is_rtr}, {msg.is_extended}, {msg.is_error}, {msg.dlc}, {msg.data}")
    
    # def publish(self):
    #     msg = Bool()
    #     msg.data = self.is_active
    #     self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = Ultrasonic()

    rclpy.spin(lidar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
