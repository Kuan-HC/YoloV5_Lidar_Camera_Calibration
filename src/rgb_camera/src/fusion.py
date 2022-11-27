#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

# msg sync
import message_filters
import queue

import numpy as np
import cv2
import sys

from pcl_project import pcl_projection

# For yolo detection
sys.path.append("/home/kuan/Lidar_Projection/src/yolov5")
from utils.plots import colors

import torch



class fusion:
    def __init__(self, max_depth = 50., min_depth = 0.5, img_width = 1280, img_height = 1024): 

        self.image  = message_filters.Subscriber("/rgb_camera/image", Image)
        self.lidar  = message_filters.Subscriber("/ouster/points", PointCloud2)
        # TimeSynchronizer is not working for our devices
        self.sync = message_filters.ApproximateTimeSynchronizer([self.image, self.lidar], queue_size=5, slop=0.05)
        self.sync.registerCallback(self.callback)
        
        self.bridge = CvBridge()
        self.projection = pcl_projection()
        self.max_depth = max_depth
        self.min_depth = min_depth

        self.pub = rospy.Publisher('fusion/image', Image, queue_size=1)
        self.counter = 0
        self.dataQue = queue.Queue(10)
        self.img_width = img_width
        self.img_height = img_height

        self.init_model()

    def init_model(self):
        """
        Loads Yolo5 model from pytorch hub.
        :return: Trained Pytorch model.
        """
        self.model = torch.hub.load('/home/kuan/Lidar_Projection/src/yolov5', 'custom',
                    path='/home/kuan/Lidar_Projection/src/rgb_camera/model/yolov5m.pt', source='local')
        
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device) 
        print("[+] Load Yolov5")

    def callback(self, msg_img, msg_lidar ): 
        self.dataQue.put([msg_img, msg_lidar])
        # For debug
        # print("[+] image:", msg_img.header.stamp.to_sec())
        # print("[+] lidar:", msg_lidar.header.stamp.to_sec())     

    def run(self, rad):
        classes = self.model.names # Get the name of label index
        print("[+] Lidar data projection")
        print("[+] Object detection")

        while not rospy.is_shutdown():
            #print("[+] queue size: {val}".format(val = self.dataQue.qsize()))
            data = self.dataQue.get()
        
            msg_img = data[0]
            msg_lidar = data[1]
            cv_image = self.bridge.imgmsg_to_cv2(msg_img, "8UC3")

            results = self.model(cv_image)
            labels = results.xyxyn[0][:, -1].cpu().numpy()
            cords = results.xyxyn[0][:, :-1].cpu().numpy()
            
            # Point cloud projection on images save distance in array
            distArray = np.zeros((self.img_width, self.img_height), dtype = np.float32)
            for point in pc2.read_points(msg_lidar, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
                if point[0] >= self.min_depth and point[0] <= self.max_depth:
                    X,Y = self.projection.getXY(point)
                    if X >= 0 and X < self.img_width and Y >= 0 and Y < self.img_height:
                        distArray[X][Y] = point[0]
                        bgr = self.projection.getColor(point[0])
                        cv2.circle(cv_image, (X, Y), 1, bgr, -1)  # position, radius, color thickness(-1 fill) 
            
            for (label,cord) in zip(labels, cords):
                    label = int(label)
                    # If score is less than 0.2 we avoid making a prediction.
                    '''
                    pretrained yolo5m classes: For demo just select the first 7 object type
                    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck'
                    '''
                    if cord[4] < 0.5  or label > 7: 
                        continue
                    
                    x1 = int(cord[0]*self.img_width)
                    y1 = int(cord[1]*self.img_height)
                    x2 = int(cord[2]*self.img_width)
                    y2 = int(cord[3]*self.img_height)
                    bgr = colors(label, True) # color of the box
                    x_cen = int(x1 + x2 >> 1)
                    y_cen = int(y1 + y2 >> 1)
                    
                    # search in a 10 x 10 square for dist info
                    x_min = max(0, x_cen - rad)
                    x_max = min(self.img_width, x_cen + rad)
                    y_min = max(0, y_cen - rad)
                    y_max = min(self.img_height, y_cen + rad)
                    
                    dist = []
                    for x in range(x_min, x_max):
                        for y in range(y_min, y_max):
                            if distArray[x][y] > 0.0:
                                dist.append(distArray[x][y])
                    if len(dist) != 0:
                        obj_dist = np.median(dist)
                    else:
                        obj_dist = 0.0

                    label_font = cv2.FONT_HERSHEY_DUPLEX #Font for the label.
                    cv2.putText(cv_image, "{:.2f} m".format(obj_dist), (x_cen - 10, y_cen -10 ), label_font, 1 , bgr, 2) #Put a label over box                                      
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), bgr, 2) #Plot the boxes
                    cv2.circle(cv_image, (x_cen, y_cen), 10, bgr, 2)  # position, radius, color thickness(-1 fill) 
                    cv2.putText(cv_image, "{}: {:.2f}".format(classes[label], cord[4]), (x1, y1 -10 ), label_font, 1, bgr, 2) #Put a label over box


            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            header = Header()
            header.stamp = rospy.Time.now()
            msg.header = header
            self.pub.publish(msg)

            while(self.dataQue.qsize() > 1):
                self.dataQue.get()       
    
if __name__ == '__main__':
    
    rospy.init_node('fusion', anonymous=True)  
    sensor_fusion = fusion() 
    sensor_fusion.run(10)

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