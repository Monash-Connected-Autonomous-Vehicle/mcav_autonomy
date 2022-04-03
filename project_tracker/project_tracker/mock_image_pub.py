#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import glob 
import time

import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

class MockImagePub(Node):

    def __init__(self):
        super(MockImagePub, self).__init__('MockImagePub')
        self._publisher = self.create_publisher(Image, '/camera', 10)
        self.ImagePath = glob.glob('/home/mcav/DATASETS/streetViewImages/*.jpg')
        self.FrameCount = 1

        while True:
            self.publish_Image()
            self.get_logger().info('sent frame ' + str(self.FrameCount))
            self.FrameCount += 1
            time.sleep(1)

    def publish_Image(self):
        """publisher"""
        
        if self.ImagePath:
            msg = self.cv2ImageToRosImage(self.ImagePath.pop())
            self._publisher.publish(msg)
            
        else: # restart
            self.ImagePath = glob.glob('/home/mcav/DATASETS/streetViewImages/*.jpg')
            self.get_logger().info("Restarting from beginning")
            self.FrameCount = 1
    
    def cv2ImageToRosImage(self, imgPath):
        """Method to convert openCV images to ROS Image message"""
        cv2_img = cv.imread(imgPath)

        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="passthrough")
        image_message.header.frame_id = 'velodyne'

        return image_message


def main(args=None):
    rclpy.init(args=args)

    mockImagePub = MockImagePub()

    try:
        rclpy.spin(mockImagePub)
    except KeyboardInterrupt:
        mockImagePub.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    mockImagePub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
