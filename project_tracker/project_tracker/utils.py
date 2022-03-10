from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

def numpy_2_PCL2(cloud, timestamp, frame_id='velodyne'):
    header = Header()
    header.frame_id = frame_id
    header.stamp = timestamp
    
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='r', offset=12, datatype=PointField.FLOAT32, count=1)
    ]

    pcl2_msg = point_cloud2.create_cloud(header, fields, cloud)
    
    return pcl2_msg