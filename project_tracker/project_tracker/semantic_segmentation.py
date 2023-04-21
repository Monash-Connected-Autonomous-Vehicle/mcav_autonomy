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
from sensor_msgs.msg import CompressedImage

class SemanticSegmentation(Node):
    def __init__(self):
        super(SemanticSegmentation, self).__init__('semantic_segmentation')
        self._subscriber = self.create_subscription(CompressedImage, '/gmsl_camera/port_0/cam_0/image_raw/compressed', self._callback, 10)
        self._lane_publisher = self.create_publisher(CompressedImage, '/lane_image', 10)
        self._dz_publisher = self.create_publisher(CompressedImage, '/drivable_zone_image', 10)   
        
        self.get_logger().set_level(logging.DEBUG)
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s') 
        # self.model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True, source='local')
        self.model = torch.hub.load('hustvl/yolop', 'yolop')
        self.get_logger().debug("At the innit function")


    def _callback(self, msg: CompressedImage):
        msg.header.frame_id = "camera"
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image_12ch = np.repeat(cv_image[:, :, np.newaxis, :], 12, axis=2)
        cv_image_tensor = torch.from_numpy(cv_image_12ch[..., :3]).permute(0, 3, 1, 2).float()/255.0
        _, dz_out, lane_out = self.model(cv_image_tensor)
        self.get_logger().debug("model loaded")

        # Reversing colour
        dz_image_array = np.array(dz_out.render()[0])[:,:,::-1]
        
        dz_image_message = bridge.compressed_imgmsg_to_cv2(dz_image_array, desired_encoding="passthrough")
        dz_image_message.header.frame_id = msg.header.frame_id

        lane_image_array = np.array(lane_out.render()[0])[:,:,::-1]
        lane_image_message = bridge.compressed_imgmsg_to_cv2(lane_image_array, desired_encoding="passthrough")
        lane_image_message.header.frame_id = msg.header.frame_id
        self.get_logger().debug("Reversing colours")
        
        # Publishing segmented image
        self._lane_publisher.publish(dz_image_message)
        self._dz_publisher.publish(lane_image_message)
        self.get_logger().debug("Publishing images")

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