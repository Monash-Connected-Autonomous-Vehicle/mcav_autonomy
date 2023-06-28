#!/usr/bin/env python3
import rclpy
import logging
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo

class ImageModifier(Node):
    def __init__(self):
        super().__init__('image_modifier')
        self.subscription = self.create_subscription(CompressedImage, '/gmsl_camera/port_0/cam_0/image_raw/compressed', self.image_callback, 10)
        self.publisher = self.create_publisher(CompressedImage, '/modified_cam/compressed', 10)
        self.info_publisher = self.create_publisher(CameraInfo, '/modified_cam/camera_info', 10)

    def image_callback(self, msg:CompressedImage):
        self.get_logger().set_level(logging.DEBUG)
        msg.header.frame_id = 'camera'
        self.publisher.publish(msg)
        info = CameraInfo()
        self.info_publisher.publish(info)
        self.get_logger().debug("Publishing images in the camera frame! ")

def main(args=None):
    rclpy.init(args=args)
    image_modifier = ImageModifier()
    rclpy.spin(image_modifier)
    image_modifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()