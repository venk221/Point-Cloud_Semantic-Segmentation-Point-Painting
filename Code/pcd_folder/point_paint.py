
import numpy as np
import open3d as o3d
import os
import calibration as ca
import utils as ut
import matplotlib.pyplot as plt
from argparse import ArgumentParser
import cv2
import matplotlib.pyplot as plt
from distutils.util import strtobool

def main():

    """
    Does Point Painting of all PCD's based on semantic segmentation done by Deeplabv3
    """
    

    #Get all calibration parameters from the calib.txt given in the repo. Repo for KITTI360
    calib = ca.CalibrationData('./calib.txt')
    R = calib.R0
    P = calib.P
    K = calib.K
    D = calib.D
    Tr_cam_to_lidar = calib.Tr_cam_to_lidar
    Tr_lidar_to_cam = ut.transform_velo_to_cam(R, Tr_cam_to_lidar)
    P_lidar_to_cam = ut.projection_velo_to_cam(R, Tr_lidar_to_cam,P)

    ##Get path for data folder containing rgb and pcd files
    print("Initializing the pipeline")

    pcd_data = os.listdir('./pcd/')
    pcd_data.sort()

    image_data=os.listdir('./images')
    image_data.sort()

    image_seg_data=os.listdir('./images_seg')
    image_seg_data.sort()


    for idx in range(len(pcd_data)):

        # print('./'+image_data[idx])

        rgb_image=cv2.imread('./images/'+image_data[idx])
        fused_image=rgb_image.copy()

        rgb_image_segmented=cv2.imread('./images_seg/'+image_seg_data[idx])
        
        pcd=np.asarray(o3d.io.read_point_cloud('./pcd/'+pcd_data[idx]).points)
        ## Only taking lidar points in front of the camera.
        point_cloud=pcd[pcd[:,0]>=0]

        pts_2D,depth, pts_3D_img = ut.project_lidar_on_image(P_lidar_to_cam, point_cloud, (rgb_image.shape[1], rgb_image.shape[0]))

        # Number of lidar points projected on image
        N = pts_3D_img.shape[0]

        ##Class color label that is to be projected on the point cloud.
        semantic = np.zeros((N,3), dtype=np.float32)
        
        print(idx)

        for i in range(pts_2D.shape[0]):
        

            x = np.int32(pts_2D[i, 0])
            y = np.int32(pts_2D[i, 1])
            

            colour = np.float64(rgb_image_segmented[y, x]) 
            
            pt = (x,y)
            cv2.circle(fused_image, pt, 2, color=colour, thickness=1)

            ###Save this class colour
            semantic[i] = rgb_image_segmented[y,x]

        stacked_img = np.vstack((rgb_image, rgb_image_segmented,fused_image))
        fname=image_data[idx].split('.')[0]
        cv2.imwrite(f'./images_fused/{fname}_fused.png',stacked_img)


        ### Actually adding the color to the point cloud
        rgb_pointcloud = np.hstack((pts_3D_img[:,:3], semantic))

        #Get RGB values from pointcloud
        # semantics  = rgb_pointcloud[:, 3]
        #Get xyz values from pointcloud
        xyz = rgb_pointcloud[:, 0:3]

        visualizer = o3d.visualization.Visualizer()
        pcd = o3d.geometry.PointCloud()
        visualizer.add_geometry(pcd)

        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(semantic)

        #o3d.visualization.draw_geometries([pcd])

        o3d.io.write_point_cloud(f'./pcd_painted/{fname}_painted.pcd',pcd)
                
                
if __name__ == '__main__':
    main()
