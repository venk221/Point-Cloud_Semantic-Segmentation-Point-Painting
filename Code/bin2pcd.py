import numpy as np
import struct
import sys
import open3d as o3d
import os
 
 
def bin_to_pcd(binFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd
 
def main(binFileName, pcdFileName):
    pcd = bin_to_pcd(binFileName)
    o3d.io.write_point_cloud(pcdFileName, pcd)
 
if __name__ == "__main__":

    data_directory='../KITTI-360/data_3d_raw/2013_05_28_drive_0018_sync/velodyne_points/data/'
    pcd_data = os.listdir(data_directory)
    pcd_data.sort()

    for i in range(len(pcd_data)):
        # num=0000000000
        if i<50:

            a = data_directory+pcd_data[i]
            fname=pcd_data[i].split('.')[0]
            b = f'./pcd_folder/pcd/{fname}.pcd'
            main(a, b)
