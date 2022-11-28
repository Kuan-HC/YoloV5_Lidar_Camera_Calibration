import open3d as o3d
import numpy as np
import argparse

def get_args():
    parser = argparse.ArgumentParser(description='pcd_visual')
    parser.add_argument('--file', type=str, help='file path')
    args = parser.parse_args()
    return args

args = get_args()

pcd = o3d.io.read_point_cloud(args.file)
o3d.visualization.draw_geometries([pcd])