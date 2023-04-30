#!/bin/sh
cd Code
python3 bin2pcd.py
cd pcd_folder
python3 icp.py --path ./pcd/
cd ../DeepLabV3Plus-Pytorch
python3 semantic_segmentation.py
cd ../pcd_folder
python3 point_paint.py
python3 icp.py --path ./pcd_painted/
