#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

import numpy as np
import glob 
import time

import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class MockImagePub(Node):

    def __init__(self, image_path, frame_id):
        super(MockImagePub, self).__init__('MockImagePub')
        
        self.Frame_Id = frame_id
        self.Image_path = image_path
        self._publisher = self.create_publisher(Image, '/camera', 10)
        self.Images = glob.glob(self.Image_path + '*.jpg')
        self.FrameCount = 1

        while True:
            self.publish_Image()
            self.get_logger().info('sent frame ' + str(self.FrameCount))
            self.FrameCount += 1
            time.sleep(1)

    def publish_Image(self):
        """publisher"""
        
        if self.Images:
            msg = self.cv2ImageToRosImage(self.Images.pop())
            self._publisher.publish(msg)
            
        else: # restart
            self.Images = glob.glob(self.Image_path + '*.jpg')
            self.get_logger().info("Restarting from beginning")
            self.FrameCount = 1
    
    def cv2ImageToRosImage(self, imgPath):
        """Method to convert openCV images to ROS Image message"""
        cv2_img = cv.imread(imgPath)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="passthrough")
        image_message.header.frame_id = self.Frame_Id

        return image_message


def main(image_path, frame_id, args=None):
    rclpy.init(args=args)

    mockImagePub = MockImagePub(image_path, frame_id)

    try:
        rclpy.spin(mockImagePub)
    except KeyboardInterrupt:
        mockImagePub.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    mockImagePub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    args = sys.argv
    main(image_path=args[1], frame_id=args[2])
