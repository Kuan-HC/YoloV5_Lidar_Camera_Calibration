#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
This file is based on the code from eRCaGuy_dotfiles: https://github.com/ElectricRCAircraftGuy/eRCaGuy_dotfiles
Sampling the ROS bag file with a configurable parameter of sampling rate.
"""
import sensor_msgs.point_cloud2 as pc2
import rosbag
import os
import sys
from tqdm import tqdm
import struct
import numpy as np
import cv2
from cv_bridge import CvBridge
import argparse

RGB_IMAGE_TOPIC = '/rgb_camera/image'

#select converted or raw point cloud data: /rslidar_points  or /convert_ptc

def get_args():
    parser = argparse.ArgumentParser(description='Rosbag sampler')
    parser.add_argument('--rate', type=int, default=10, help='sampling rate (1 sample to take per X frames)')
    parser.add_argument('--output_dir', type=str, default=os.curdir, help='directory to save the output files')
    parser.add_argument('--pcd',  type=str, default='/ouster/points', help='select point cloud topic')
    parser.add_argument('--bag', type=str, help='Rosbag to read and sample')
    args = parser.parse_args()
    return args

def decode_pointcloud(msg, fields):
    data = msg.data
    is_bigendian = msg.is_bigendian
    point_step = msg.point_step
    point_cloud = []
    for i in range(0, len(data), point_step):
        d = data[i:i+point_step]
        point = []
        for field in ['x', 'y', 'z', 'reflectivity']:
            offset, format_str, num_bytes = fields[field]
            if not is_bigendian:
                format_str = '<'+format_str
            value = struct.unpack(format_str, d[offset:offset+num_bytes])
            point.append(value)
        point_cloud.append(point)
    return np.array(point_cloud).reshape(-1, 4)

def verify_outdir(outdir):
    if os.path.exists(outdir) and len(os.listdir(outdir))>0:
        #raise RuntimeError(f'Output directory not empty: {outdir}')
        raise RuntimeError("Output directory not empty: {path}".format(path=outdir))
    elif not os.path.exists(outdir):
        os.makedirs(outdir)


def parse_bag(args):
    total_count = 0
    bag = rosbag.Bag(args.bag)
    topics = list(bag.get_type_and_topic_info().topics.keys())
    print("[+] Topics in this bag file")
    print(topics)

    if RGB_IMAGE_TOPIC not in topics:
        print("[+] Can't find rgb image")  
        sys.exit()
    elif args.pcd not in topics:
        print("[+] Can't find {pcd_topic} topic".format(pcd_topic = args.pcd))  
        sys.exit()
    
    output_subdir = os.path.basename(args.bag).split('.')[0]
    # build output_dir
    point_cloud_bin = "{output_dir}/{sub_dir}/point_cloud_bin".format(output_dir=args.output_dir,sub_dir=output_subdir)
    verify_outdir(point_cloud_bin)
    point_cloud_pcd = "{output_dir}/{sub_dir}/point_cloud_pcd".format(output_dir=args.output_dir,sub_dir=output_subdir)
    verify_outdir(point_cloud_pcd)
    meta_root = "{output_dir}/{sub_dir}/meta".format(output_dir=args.output_dir,sub_dir=output_subdir)
    verify_outdir(meta_root)    
    rgb_root = "{output_dir}/{sub_dir}/rgb".format(output_dir=args.output_dir,sub_dir=output_subdir)
    verify_outdir(rgb_root)

    # stat rgb and time stamps:
    rgb_time_stamps = {}
    for _, m, t in tqdm(bag.read_messages(RGB_IMAGE_TOPIC), desc='Stat rgb time stamps'):
        rgb_time_stamps[m.header.stamp.to_sec()]=t
    
    img_times = np.array(list(rgb_time_stamps.keys()))
    img_times.sort()


    count, sample_rate = -1, args.rate

    ###### Start read message #####
    bridge = CvBridge()
    for _, msg, t in tqdm(bag.read_messages(args.pcd), desc='Parse Rosbag'):
        # sampling
        count += 1
        if count%sample_rate != 0:
            continue

        # Interpret Point Cloud
        timestamp = msg.header.stamp.to_sec()
        point_cloud = np.array(list(pc2.read_points(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True)))

        # save pcd file
        pc_pcd_out_file = "{pc_root}/{total_count}.pcd".format(pc_root=point_cloud_pcd, total_count=total_count)
        pcd_length = len(point_cloud)
        print("length", pcd_length)
        np.savetxt(pc_pcd_out_file, point_cloud)
        with open(pc_pcd_out_file, "r+") as f:
            old = f.read()
            f.seek(0)
            f.write("# .PCD v.7 - Point Cloud Data file format\n")
            f.write("VERSION .7\n")
            f.write("FIELDS x y z intensity\n" )
            f.write("SIZE 4 4 4 4\n" )
            f.write("TYPE F F F F\n" )
            f.write("COUNT 1 1 1 1\n" )
            f.write("WIDTH {width}\n".format(width = pcd_length))
            f.write("HEIGHT 1\n" )
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n" )
            f.write("POINTS {width}\n".format(width = pcd_length))
            f.write("DATA ascii\n" )
            f.write(old)
        '''     
        if args.pcd == '/rslidar_points':
            min_intensity = min(point_cloud[:,3])
            max_intensity = max(point_cloud[:,3])
            tmp = (point_cloud[:,3] - min_intensity)/(max_intensity-min_intensity)
            point_cloud[:,3]=tmp
        '''

        # get nearest rgb image        
        idx = np.searchsorted(img_times, timestamp, side='left')
        nearest_time = img_times[idx-1] if abs(img_times[idx-1]-timestamp)<abs(img_times[idx]-timestamp) else img_times[idx]
        _, img_msg, _ = next(bag.read_messages(RGB_IMAGE_TOPIC, rgb_time_stamps[nearest_time]))
        rgb_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        rgb_time_diff = timestamp - nearest_time

        # store to file, PointCloud: .bin, Image: .png (together with meta data)
        meta_data = "Point cloud time:: {time}".format(time=timestamp)
        pc_bin_out_file = "{pc_root}/{total_count}.bin".format(pc_root=point_cloud_bin, total_count=total_count)
        meta_out_file = "{meta_root}/{total_count}.txt".format(meta_root=meta_root, total_count=total_count)

        point_cloud.astype('float32').tofile(pc_bin_out_file)   # save in binary file, following Kitti format
        
        # rgb output
        rgb_out_file = "{img_root}/{total_count}.png".format(img_root=rgb_root, total_count=total_count)
        cv2.imwrite(rgb_out_file, rgb_img)

        meta_data += "\nRGB time diff: {img_time_diff}".format(img_time_diff=rgb_time_diff)      
        
        with open(meta_out_file, 'w') as f:
            f.write(meta_data)

        total_count+=1

if __name__=='__main__':
    args = get_args()
    parse_bag(args)   