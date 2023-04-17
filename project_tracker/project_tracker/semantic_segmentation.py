#!/usr/bin/env python3

# Official documentation - https://pytorch.org/hub/hustvl_yolop/
# Requirements - pip install -qr https://github.com/hustvl/YOLOP/blob/main/requirements.txt 

# Libraries and dependencies
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
        self._lane_publisher = self.create_publisher(Image, '/lane_image', 10)
        self._dz_publisher = self.create_publisher(Image, '/drivable_zone_image', 10)
        
        self.get_logger().set_level(logging.DEBUG)
        self.model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True, source='local')

    def _callback(self, msg: Image):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        _, dz_out, lane_out = self.model(cv_image)

        # Reversing colour
        dz_image_array = np.array(dz_out.render()[0])[:,:,::-1]
        dz_image_message = bridge.cv2_to_imgmsg(dz_image_array, encoding="passthrough")
        dz_image_message.header.frame_id = msg.header.frame_id

        lane_image_array = np.array(lane_out.render()[0])[:,:,::-1]
        lane_image_message = bridge.cv2_to_imgmsg(lane_image_array, encoding="passthrough")
        lane_image_message.header.frame_id = msg.header.frame_id

        # Publishing segmented image
        self._publisher.publish(dz_image_message)
        self._publisher.publish(lane_image_message)

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