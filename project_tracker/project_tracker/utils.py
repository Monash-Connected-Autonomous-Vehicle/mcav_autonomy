from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2 as PCL2

import numpy as np
from itertools import chain
import struct
from random import randint
import ctypes

def numpy_2_PCL2(cloud, timestamp, frame_id='velodyne', reflectance=True):
    """
    Utility function to take in a numpy pointcloud array and return a PCL2 message for publishing in ROS2.

    Parameters
    ----------
    cloud : np.array
        Expected shape of [n, 3 or 4]. 3 if xyz, 4 if xyzr 
    timestamp : ROS2 timestamp
    frame_id : str
        frame ID reference for use in conjunction with other pointclouds. i.e. where the data is relative 
        to other clouds in space. Allows for transformations between frames
    reflectance : bool
        Whether reflectance is being published in this message or if it has been discarded
    Returns
    -------
    PCL2 Message : PCL2 message for use with ROS topics.
    """
    header = Header()
    header.frame_id = frame_id
    header.stamp = timestamp
    
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if reflectance:
        fields.append(PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1))

    pcl2_msg = point_cloud2.create_cloud(header, fields, cloud)
    
    return pcl2_msg


def PCL2_2_numpy(pcl2_msg, reflectance=True):
    """
    Utility function to take in a PCL2 message from ROS2 topic and convert it to a numpy pointcloud array.

    Parameters
    ----------
    pcl2_msg : PCL2 Message
        PCL2 message from a ROS topic.
    reflectance : bool
        Whether reflectance is being received in this message or if it has been discarded
    Returns
    -------
    np.array : shape [n, 3 or 4]. 3 if xyz, 4 if xyzr
    """
    cloud_generator = point_cloud2.read_points(pcl2_msg)
    # convert cloud to numpy array
    cloud_np = np.fromiter(chain.from_iterable(cloud_generator), np.float32)
    if reflectance:
        return cloud_np.reshape((-1, 4))
    else:
        return cloud_np.reshape((-1, 3))

def create_colour_list():
    """Create list of colours in ROS float format for use in ROS PCL2 messages"""
    colour_list = []
    for i in range(300):
        colour_list.append(rgb_to_float(random_color_gen()))
    return colour_list

def random_color_gen():
    """ Generates a random color
    
        Args: None
        
        Returns: 
            list: 3 elements, R, G, and B
    """
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]

def rgb_to_float(color):
    """ 
    Converts an RGB list to the packed float format used by PCL
    
    From the PCL docs:
    "Due to historical reasons (PCL was first developed as a ROS package),
        the RGB information is packed into an integer and casted to a float"

    Args:
        color (list): 3-element list of integers [0-255,0-255,0-255]
        
    Returns:
        float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


def pcl_to_ros(pcl_array, timestamp):
    """ Converts a pcl PointXYZRGB to a ROS PointCloud2 message
    
        Args:
            pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
            
        Returns:
            PointCloud2: A ROS point cloud
    """
    ros_msg = PCL2()

    ros_msg.header.stamp = timestamp
    ros_msg.header.frame_id = "velodyne"

    ros_msg.height = 1
    ros_msg.width = pcl_array.size

    ros_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    ros_msg.is_bigendian = False
    ros_msg.point_step = 32
    ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
    ros_msg.is_dense = False
    buffer = []

    for data in pcl_array:
        s = struct.pack('>f', data[3])
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value

        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = (pack & 0x000000FF)

        buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    ros_msg.data = b"".join(buffer)

    return ros_msg