#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
import sensor_msgs
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import numpy as np

from message_filters import TimeSynchronizer
import message_filters

from pcl_project import pcl_projection

from cv_bridge import CvBridge

class fusion:
    def __init__(self, max_depth = 50., min_depth = 3.): 

        self.image  = rospy.Subscriber("/rgb_camera/image", Image, self.callback, queue_size = 5)
        
        self.bridge = CvBridge()
        self.projection = pcl_projection()
        self.max_depth = max_depth
        self.min_depth = min_depth

        self.pub = rospy.Publisher('fusion/image', Image, queue_size=1)
        self.counter = 0
        

    def callback(self, img): 
        cv_image = self.bridge.imgmsg_to_cv2(img, "8UC3")
        i = self.counter % 5
        if i == 0 :
            lidar_msg = rospy.wait_for_message("/ouster/points", PointCloud2,10)
        
        
            for point in pc2.read_points(lidar_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
                if point[0] >= self.min_depth and point[0] <= self.max_depth:
                    X,Y = self.projection.getXY(point)
                    bgr = self.projection.getColor(point[0])
                    cv2.circle(cv_image, (X, Y), 1, bgr, -1)  # position, radius, color thickness(-1 fill) 
                
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            header = Header()
            header.stamp = rospy.Time.now()
            msg.header = header
            self.pub.publish(msg)

        self.counter += 1
       
    


if __name__ == '__main__':

    rospy.init_node('fusion', anonymous=True)  
    sensor_fusion = fusion()     

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


    
'''       
                
if __name__ == '__main__':
    rospy.init_node('pcl_filter', anonymous=True)
    print("[+] Point Cloud Filter Node")  

    #rospy.Subscriber("/ouster/points", PointCloud2, callback, queue_size=1)  
    rospy.Subscriber("/rgb_camera/image", Image, callback, queue_size=1) 
    
    rospy.spin()
'''