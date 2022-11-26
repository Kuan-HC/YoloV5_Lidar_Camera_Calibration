import os
import sys
from ctypes import *
import cv2
import numpy as np
import argparse

abs = os.path.abspath(__file__) #current path
parent_path = os.path.dirname(abs)
parent_path = os.path.dirname(parent_path)
package_path = parent_path


class pcl_projection:
    def __init__(self):
        self.init_parameters()             

    def init_parameters(self):
        self.intrinsic = np.loadtxt("{path}/Calibration/mtx.txt".format(path = package_path))
        self.extrinsic = np.loadtxt("{path}/Calibration/extrinsic.txt".format(path = package_path))
        print("extrinsic", self.extrinsic)
        self.transM = np.dot(self.intrinsic, self.extrinsic)  

    def getXY(self, msg):
        coord = np.zeros((4,1), dtype = np.float32)
        coord[0] = msg[0]
        coord[1] = msg[1]
        coord[2] = msg[2]
        coord[3] = 1
        temp = np.dot(self.transM, coord)
        x = temp[0] / temp[2]
        y = temp[1] / temp[2]

        return int(x), int(y)
        

    def pcd_to_img(self, img_path, pcd_path):
        
        pcd = np.loadtxt(pcd_path)
        print("Pcd length", len(pcd))
        
        frame = cv2.imread(img_path)
        width = frame.shape[1]
        height = frame.shape[0]
        print("Image height: {}, widht: {}".format(height, width))

        cv2.namedWindow('img', cv2.WINDOW_NORMAL)        
        
        count = 0
        for row in pcd:
            if row[0] > 1.0:
                X,Y = self.getXY(row)
                #print("count:{}, X:{} U:{}".format(count, X,Y))
                cv2.circle(frame, (X, Y), 1, (0, 0, 255), -1)  # position, radius, color thickness(-1 fill) 
                count += 1
            
        print("Total {} points".format(count))
        cv2.imshow("img", frame)

        key = cv2.waitKey(0)
        if key == 27:
            cv2.destroyAllWindows()


def get_args():
    parser = argparse.ArgumentParser(description='pcd_visual')
    parser.add_argument('--img', type=str, default='sample.png',help='img path')
    parser.add_argument('--pcd', type=str, default='sample.pcd',help='pcd path')
    args = parser.parse_args()
    return args



if __name__ == "__main__":        
    args = get_args()
    pcl = pcl_projection()
    pcl.pcd_to_img(args.img, args.pcd)