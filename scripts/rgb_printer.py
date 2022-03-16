#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes

def callback(data):
    gen = pc2.read_points(data)
    int_data = list(gen)

    for x in int_data:
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        print(r)
        print(g)
        print(b)
        print('-------')
    
def listener():

    rospy.init_node('sub_pcl', anonymous=True)
    rospy.Subscriber("ground_proj_plane", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()