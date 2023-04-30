import open3d as o3d
import numpy as np
import copy
import os
from argparse import ArgumentParser
# import pry

def draw_registration_result(source, target, transformation):

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp])


def point_to_point_icp(source, target, threshold, trans_init, transformation_list):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation, "\n")
    transformation_list.append(reg_p2p.transformation)
    # draw_registration_result(source, target, reg_p2p.transformation)


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument('--path', help='path file for data')

    args = parser.parse_args()

    base_path = args.path
    

    pcd_data = os.listdir(base_path)
    pcd_data.sort()
    transformation_list = []
    for i in range(len(pcd_data)-1):
        source = o3d.io.read_point_cloud(base_path+pcd_data[i])
        target = o3d.io.read_point_cloud(base_path+pcd_data[i+1])
        threshold = 0.2
        trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                                [-0.139, 0.967, -0.215, 0.7],
                                [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    # draw_registration_result(source, target, trans_init)

        # print("Initial alignment")
        # evaluation = o3d.pipelines.registration.evaluate_registration(
        #     source, target, threshold, trans_init)
        # print(evaluation, "test\n")

        point_to_point_icp(source, target, threshold, trans_init, transformation_list)

    for i in range(len(transformation_list)-1, 0, -1):
        if i==(len(transformation_list)-1):
            source= o3d.io.read_point_cloud(base_path+pcd_data[i-1])

        target = o3d.io.read_point_cloud(base_path+pcd_data[i])
        target.transform(transformation_list[i])
        source+=target


    if base_path=='./pcd/':
        o3d.io.write_point_cloud('final_pointcloud_initial.pcd',source)
    else:
        o3d.io.write_point_cloud('final_pointcloud_painted.pcd',source)

    

    o3d.visualization.draw_geometries([source])



