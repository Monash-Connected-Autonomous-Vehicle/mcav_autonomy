#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import logging
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

import torch


class ObjectDetection(Node):
    def __init__(self):
        super(ObjectDetection, self).__init__('object_detection')
        self.subscription = self.create_subscription(Image, '/image_raw', self._callback, 10)
        self._publisher = self.create_publisher(Image, '/object_detected_image', 10)
        self.get_logger().set_level(logging.DEBUG)

        # Model
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
        self.model = torch.hub.load('yolov5', 'custom', path='/home/mcav/mcav_ws/src/mcav_autonomy/yolov5s.pt', source='local')

    def _callback(self, msg: Image):
        # convert ros2 image to cv_image to allow for processing through yolo
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Inference
        results = self.model(cv_image)
        
        # print objects found in the console
        # results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
        
        # numpy array representing the classified image
        image_array = np.array(results.render()[0])[:,:,::-1] # reverse bgr to rgb

        image_message = bridge.cv2_to_imgmsg(image_array, encoding="passthrough")
        image_message.header.frame_id = msg.header.frame_id

        self._publisher.publish(image_message)


def main(args=None):
    rclpy.init(args=args)

    object_detection = ObjectDetection()

    try:
        rclpy.spin(object_detection)
    except KeyboardInterrupt:
        object_detection.get_logger().debug("Keyboard interrupt")

    # destroy node explicity
    object_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

