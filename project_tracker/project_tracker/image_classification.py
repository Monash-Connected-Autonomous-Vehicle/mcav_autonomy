#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import logging
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

import torch


class ImageClassification(Node):
    def __init__(self):
        super(ImageClassification, self).__init__('imageClassification')
        
        self.subscription = self.create_subscription(Image, '/camera', self._callback, 10)
        self._publisher = self.create_publisher(Image, '/classified_image', 10)

        self.get_logger().set_level(logging.DEBUG)

    def _callback(self, msg: Image):
        # Model
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom

        # convert ros2 image to cv_image to allow for processing through yolo
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Inference
        results = model(cv_image)
        
        # print objects found in the console
        results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
        
        # numpy array representing the classified image
        image_array = np.array(results.render())[0]

        image_message = bridge.cv2_to_imgmsg(image_array, encoding="passthrough")

        self._publisher.publish(image_message)


def main(args=None):
    rclpy.init(args=args)

    imageClassification = ImageClassification()

    try:
        rclpy.spin(imageClassification)
    except KeyboardInterrupt:
        imageClassification.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    imageClassification.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

