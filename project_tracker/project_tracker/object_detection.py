#!/usr/bin/env python3

# Libraries and dependencies
import rclpy
import torch
import logging
import cv2 as cv
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

class ObjectDetection(Node):
    def __init__(self):
        super(ObjectDetection, self).__init__('object_detection')
        self.get_logger().set_level(logging.DEBUG)
        self.get_logger().debug("YOLO object detection active")
        self._subscriber = self.create_subscription(CompressedImage, '/modified_cam/compressed', self._callback, 10)
        self._yolo_publisher = self.create_publisher(Image, '/yolo_image_pred', 10)

        # Loading model
        self.yolov5_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
                
        # Bridging messgaes to opencv
        self.bridge = CvBridge()

    def _callback(self, msg: CompressedImage):
        # Converting a CompressedImage ROS message into an OpenCV Mat
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Predicting on a Mat array
        yolo_out = self.yolov5_model([cv_image])
        self.get_logger().debug(f"Output shape - {yolo_out}")
        
        # Reversing colour
        yolo_image_array = np.array(yolo_out.render()[0])[:, :, [0, 1, 2]]
        # yolo_image_array = cv.cvtColor(yolo_out, cv.COLOR_BGR2RGB)
        yolo_image_array_contiguous = np.ascontiguousarray(yolo_image_array)
        yolo_image_message = self.bridge.cv2_to_imgmsg(yolo_image_array_contiguous)
        yolo_image_message.header.frame_id = msg.header.frame_id
        
        # Publishing segmented image
        self._yolo_publisher.publish(yolo_image_message)
        self.get_logger().debug("Publishing images")

def main(args = None):
    rclpy.init(args=args)
    semantic_segmentation = ObjectDetection()

    try:
        rclpy.spin(semantic_segmentation)
    except KeyboardInterrupt:
        semantic_segmentation.get_logger().debug("Keyboard interrput for semantic segmentation")
    
    semantic_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()