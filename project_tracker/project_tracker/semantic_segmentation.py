#!/usr/bin/env python3
import rclpy
import torch
import logging
import cv2 as cv
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class SemanticSegmentation(Node):
    def __init__(self):
        super(SemanticSegmentation, self).__init__('semantic_segmentation')
        self._subscriber = self.create_subscription(Image, '/image_downsampled', self._callback, 10)
        self._publisher = self.create_publisher(Image, '/segmented_image', 10)
        self.get_logger().set_level(logging.DEBUG)
        self.model = torch.hub.load('yolov8', 'custom', path='/home/mcav/mcav_ws/src/mcav_autonomy/project_tracker/project_tracker/yolov8n-seg.pt', source='local')

    def _callback(self, msg: Image):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        results = self.model(cv_image)

        # Reversing colour
        seg_image_array = np.array(results.render()[0])[:,:,::-1]
        seg_image_message = bridge.cv2_to_imgmsg(seg_image_array, encoding="passthrough")
        seg_image_message.header.frame_id = msg.header.frame_id

        # Publishing segmented image
        self._publisher.publish(seg_image_message)

def main(args = None):
    rclpy.init(args=args)
    semantic_segmentation = SemanticSegmentation()

    try:
        rclpy.spin(semantic_segmentation)
    except KeyboardInterrupt:
        semantic_segmentation.get_logger().debug("Keyboard interrput for semantic segmentation")
    
    semantic_segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()