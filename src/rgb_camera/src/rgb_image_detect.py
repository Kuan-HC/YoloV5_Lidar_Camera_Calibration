#!/usr/bin/env python3
# -- coding: utf-8 --
'''
Ros
'''
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
'''
Image stream libries
'''
import os
import sys
from ctypes import *
import cv2
import numpy as np

abs = os.path.abspath(__file__) #current path
parent_path = os.path.dirname(abs)
parent_path = os.path.dirname(parent_path)
package_path = parent_path

sys.path.append(parent_path)
from MvImport.MvCameraControl_class import *

parent_path = os.path.dirname(parent_path)
parent_path = os.path.dirname(parent_path)
parent_path = os.path.join(parent_path,'yolov5')
sys.path.append(parent_path)
from utils.plots import colors

'''
Detestion libries
'''
class RGB_Img_Detect:
    def __init__(self, source, undistort = True):
        self.source = source
        self.undistort = undistort
        self.init_camera()       
            
            
    def init_camera(self):
        if self.source == HIK_CAMERA:
            SDKVersion = MvCamera.MV_CC_GetSDKVersion()
            print ("[+] SDKVersion[0x%x]" % SDKVersion)
            deviceList = MV_CC_DEVICE_INFO_LIST()
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            
            #Enum device
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print ("[+] enum devices fail! ret[0x%x]" % ret)
                sys.exit()
 
            if deviceList.nDeviceNum == 0:
                print ("[+] Can not find HIK device!")
                sys.exit()

            print ("[+] Find %d devices! Set default device: 0" % deviceList.nDeviceNum)
            nConnectionNum = 0

            self.cap = MvCamera() 

            # Select device and create handle
            stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
 
            ret = self.cap.MV_CC_CreateHandle(stDeviceList)
            if ret != 0:
                print ("[+] create handle fail! ret[0x%x]" % ret)
                sys.exit()

            # Open device
            ret = self.cap.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                print ("[+] open device fail! ret[0x%x]" % ret)
                sys.exit()

            # en:Set trigger mode as off
            ret = self.cap.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print ("[+] set trigger mode fail! ret[0x%x]" % ret)
                sys.exit()

            # Get payload size
            stParam =  MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

            ret = self.cap.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                print ("[+] get payload size fail! ret[0x%x]" % ret)
                sys.exit()

            self.nPayloadSize = stParam.nCurValue
            # en:Start grab image
            ret = self.cap.MV_CC_StartGrabbing()
            if ret != 0:
                print ("[+] start grabbing fail! ret[0x%x]" % ret)
                sys.exit()

            #将PayloadSize的uint数据转为可供numpy处理的数据，后面就可以用numpy将其转化为numpy数组格式。
            self.data_buf = (c_ubyte * self.nPayloadSize)()
            self.stFrameInfo = MV_FRAME_OUT_INFO_EX()

            memset(byref(self.stFrameInfo), 0, sizeof(self.stFrameInfo))
            
            ret = self.cap.MV_CC_GetOneFrameTimeout(self.data_buf, self.nPayloadSize, self.stFrameInfo, 1000)
            
            if ret == 0:
                print ("[+] Hik image: Width[%d], Height[%d], PixelType[0x%x]"  % 
                    (self.stFrameInfo.nWidth, self.stFrameInfo.nHeight, self.stFrameInfo.enPixelType))
                self.width = self.stFrameInfo.nWidth
                self.height = self.stFrameInfo.nHeight
            else:
                print ("[+] get frame fail!" )
                sys.exit()

            if self.undistort == True:
                print ("[+] Load camera Intrinsic parameters!" )
                print ("[+] Image is rectified!" )
                self.mtx = np.loadtxt("{path}/Calibration/mtx.txt".format(path = package_path))
                self.newcameramtx = np.loadtxt("{path}/Calibration/newcameramtx.txt".format(path = package_path))
                self.dist = np.loadtxt("{path}/Calibration/dist.txt".format(path = package_path))
            else:
                print ("[+] Image is not rectified!" )

            
        elif self.source == WEB_CAMERA:
            self.width = 640
            self.height = 512
            fps = 50
            self.cap = cv2.VideoCapture(2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)  # 设置图像宽度
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)  # 设置图像高度
            self.cap.set(cv2.CAP_PROP_FPS , fps)

            # Checking if it's connected to the camera
            if not self.cap.isOpened():
                print("[+] Camera can't open")
                return -1
            
            print("[+] Webcam image: Width[%d], Height[%d], FPS[%d]]"  % 
                 (self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), fps))
        print("[+] Camera is ready!")

    def init_model(self):
        import torch
        """
        Loads Yolo5 model from pytorch hub.
        :return: Trained Pytorch model.
        """
        self.model = torch.hub.load('/home/kuan/Lidar_Projection/src/yolov5', 'custom',
                    path='/home/kuan/Lidar_Projection/src/rgb_camera/model/yolov5m.pt', source='local')
        
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        
    def init_ros(self):
        rospy.init_node('rgb_detect', anonymous=True)
        self.pub = rospy.Publisher('rgb_camera/image', Image, queue_size=1)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10) #10 Hz


    def read(self):
        if self.source == HIK_CAMERA: 
            ret = self.cap.MV_CC_GetOneFrameTimeout(self.data_buf, self.nPayloadSize, self.stFrameInfo, 10)
            img = np.asarray(self.data_buf)
            img = img.reshape((self.stFrameInfo.nHeight, self.stFrameInfo.nWidth, 3))
            if self.undistort == False:
                return ret^1, img
            else:
                undistort_image = cv2.undistort(img, self.mtx, self.dist, None, self.newcameramtx)
                return ret^1, undistort_image
        elif self.source == WEB_CAMERA:
            return self.cap.read()

    def show(self):
        print("[+] Show camera image!")
        self.init_ros()
        cv2.namedWindow('RGB image, Press ESC to close', cv2.WINDOW_NORMAL)

        while not rospy.is_shutdown():  
            ret, frame = self.read()
            if ret == True:
                cv2.imshow('RGB image, Press ESC to close', frame)
                
                '''
                ros send image msg
                '''
                msg = self.bridge.cv2_to_imgmsg(frame, "passthrough")
                header = Header()
                header.stamp = rospy.Time.now()
                msg.header = header
                self.pub.publish(msg)

            self.rate.sleep()
            key = cv2.waitKey(10)
            if key == 27:
                break

        self.close()             

    def detect(self):       

        print("[+] Load Yolov5!")
        self.init_ros() 
        self.init_model()

        print("[+] Object detect!")           
        cv2.namedWindow('Detect, Press ESC to close', cv2.WINDOW_NORMAL)
        classes = self.model.names # Get the name of label index

        while not rospy.is_shutdown():        
            ret, frame = self.read()
            if ret == True:
                results = self.model(frame)
                labels = results.xyxyn[0][:, -1].cpu().numpy()
                cords = results.xyxyn[0][:, :-1].cpu().numpy()
                for (label,cord) in zip(labels, cords):
                    label = int(label)
                    # If score is less than 0.2 we avoid making a prediction.
                    if cord[4] < 0.2  or label != 0:
                        continue
                    
                    x1 = int(cord[0]*self.width)
                    y1 = int(cord[1]*self.height)
                    x2 = int(cord[2]*self.width)
                    y2 = int(cord[3]*self.height)
                    bgr = colors(label, True) # color of the box
                    
                    label_font = cv2.FONT_HERSHEY_SIMPLEX #Font for the label.
                    cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2) #Plot the boxes
                    cv2.circle(frame, (x1 + x2 >> 1, y1 + y2 >> 1), 10, (0, 0, 255), 2)  # position, radius, color thickness(-1 fill) 
                    cv2.putText(frame, classes[label], (x1, y1), label_font, 2, bgr, 2) #Put a label over box.                
                cv2.imshow('Detect, Press ESC to close', frame)
                '''
                ros send image msg
                '''
                msg = self.bridge.cv2_to_imgmsg(frame, "passthrough")
                header = Header()
                header.stamp = rospy.Time.now()
                msg.header = header
                self.pub.publish(msg)

            self.rate.sleep()
            key = cv2.waitKey(10)
            if key == 27:
                break
        self.close()
             
            
    def close(self):
        if self.source == HIK_CAMERA:
            ret = self.cap.MV_CC_StopGrabbing()
            if ret != 0:
                print ("[+] stop grabbing fail! ret[0x%x]" % ret)
                del self.data_buf
                sys.exit()

            # Close device
            ret = self.cap.MV_CC_CloseDevice()
            if ret != 0:
                print ("[+] close deivce fail! ret[0x%x]" % ret)
                del self.data_buf
                sys.exit()
 
            # Destroy handle
            ret = self.cap.MV_CC_DestroyHandle()
            if ret != 0:
                print ("[+] destroy handle fail! ret[0x%x]" % ret)
                del self.data_buf
                sys.exit()
 
            del self.data_buf
            print("[+] HIK Camera closed!")
        else:
            self.cap.release()   
            print("[+] WEB Camera closed!")         
     
if __name__ == "__main__":      

    '''
    Please select your source: 
    HIK_CAMERA
    WEB_CAMERA
    Defautl HIK camera out put is rectified image, if you want raw image please select
    RGB_Img_Detect(HIK_CAMERA, False)
    '''
    device = RGB_Img_Detect(HIK_CAMERA)
    device.show()