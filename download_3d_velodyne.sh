#!/bin/bash

train_list=("2013_05_28_drive_0018_sync")

root_dir=KITTI-360
data_3d_dir=data_3d_raw

mkdir -p $root_dir
mkdir -p $root_dir/$data_3d_dir

cd $root_dir 

# 3d scans
for sequence in ${train_list[@]}; do
    zip_file=${sequence}_velodyne.zip
    wget https://s3.eu-central-1.amazonaws.com/avg-projects/KITTI-360/data_3d_raw/${zip_file}
    unzip -d ${data_3d_dir} ${zip_file} 
    rm ${zip_file}
done

# timestamps
zip_file=data_timestamps_velodyne.zip
wget https://s3.eu-central-1.amazonaws.com/avg-projects/KITTI-360/data_3d_raw/${zip_file}
unzip -d ${data_3d_dir} ${zip_file}
rm $zip_file

