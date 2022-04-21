#!/usr/bin/env python3
import sys
import time

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations

class StaticFramePublisher(Node):
    """Broadcast static transforms for Carla to StreetDrone specific frame IDs"""
    def __init__(self):
        super().__init__('static_carla_tf2_broadcaster')

        self._lidar_tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        
        while True:
            self.lidar_transform()
            self.camera_transform()
            time.sleep(0.01)

    def lidar_transform(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'velodyne'
        static_transformStamped.child_frame_id = 'ego_vehicle/lidar'
        static_transformStamped.transform.translation.x = 0.
        static_transformStamped.transform.translation.y = 0.
        static_transformStamped.transform.translation.z = 0.
        quat = tf_transformations.quaternion_from_euler(0., 0., 0.)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._lidar_tf_publisher.sendTransform(static_transformStamped)

    def camera_transform(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'velodyne'
        static_transformStamped.child_frame_id = 'ego_vehicle/rgb_front'
        static_transformStamped.transform.translation.x = 0.
        static_transformStamped.transform.translation.y = 0.
        static_transformStamped.transform.translation.z = 0.
        quat = tf_transformations.quaternion_from_euler(0., 0., 0.)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._lidar_tf_publisher.sendTransform(static_transformStamped)


def main():
    logger = rclpy.logging.get_logger('logger')

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()