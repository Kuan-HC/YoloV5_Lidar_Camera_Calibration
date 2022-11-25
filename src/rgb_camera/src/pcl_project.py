#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 15 17:36:49 2021

@author: stephen
"""
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np

'''
This node is for other lidar devices whose scan range can't be configured
'''



def callback(msg):
    # obtain point cloud data
    pcl_lst = []
    
    for p in pc2.read_points(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
        
        if p[0] > 0.0 :  # x range must  > 0.0 meter
            #print(p)
            pcl_lst.append(p)

    pcl_np = np.array(pcl_lst).reshape(-1, 4)


    ######## visualize converted point cloud
    pt_cloud = PointCloud2()
    pt_cloud.header.frame_id = msg.header.frame_id
    pt_cloud.header.stamp = msg.header.stamp

    pt_cloud.height = 1
    pt_cloud.width = pcl_np.shape[0]
        
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
        
    pt_cloud.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)]
    pt_cloud.is_bigendian = False
    pt_cloud.point_step = itemsize * 4
    pt_cloud.row_step = itemsize * 4 *pcl_np.shape[0]
    pt_cloud.is_dense = False
    pt_cloud.data = np.asarray(pcl_np, np.float32).tobytes()
        
    filter_pcl.publish(pt_cloud)  
        
                
if __name__ == '__main__':
    rospy.init_node('pcl_project', anonymous=True)
    print("[+] Point Cloud Filter Node")  

    filter_pcl = rospy.Publisher('pcl_filter', PointCloud2, queue_size=10)
    rospy.Subscriber("/ouster/points", PointCloud2, callback)  
    
    rospy.spin()
     

       
          

    
