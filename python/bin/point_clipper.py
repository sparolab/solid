import os
import natsort

import numpy as np

from tqdm import tqdm

def readScan(bin_path):
    ## KITTI
    lidar_dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
    scan        = np.fromfile(bin_path, dtype=lidar_dtype)
    points  = np.stack((scan['x'], scan['y'], scan['z'], scan['intensity']), axis = -1)
    return points

def point_path(point_dir):
    point_dir = os.path.join(point_dir)                      
    pointfile_list = os.listdir(point_dir)                                              
    pointfile_list = natsort.natsorted(pointfile_list)                                                                   
    point_fullpaths = [os.path.join(point_dir, name) for name in pointfile_list]    
    num_points = len(pointfile_list)     
    return point_fullpaths, num_points

# name =    [30, 60, 90, 120, 150,  180,  210,  240, 270, 300, 330]
# degree1 = [270, 300, 330, 360]
# degree2 = [90, 180, ]

# for i in range(len(degree1)):
bin_path = '/home/hogyun2/LiDAR_Place_Recognition_Packages/Datasets/kitti_05_sampling/bin/'
save_path = '/home/hogyun2/LiDAR_Place_Recognition_Packages/Datasets/kitti_05_690_sampling' + '/bin/'    
os.makedirs(save_path, exist_ok=True)

scan_path, num_point = point_path(bin_path)

# Point Clipping
# ===========================================================================================================
angle_threshold1 = np.radians(-90)  # 각도를 라디안으로 변환합니다.
angle_threshold2 = np.radians(60)  # 각도를 라디안으로 변환합니다.

for idx in tqdm(range(num_point)):
    scan = readScan(scan_path[idx])
    angles = np.arctan2(scan[:, 1], scan[:, 0])
    # selected_points = scan[((angles >= angle_threshold1) & (angles <= angle_threshold2)) | ((angles >= angle_threshold3) & (angles <= angle_threshold4))]
    selected_points = scan[((angles >= angle_threshold1) & (angles <= angle_threshold2))]
    selected_points.tofile(save_path+'{}.bin'.format(str(idx).zfill(6)))
    # # ===========================================================================================================

# Point Occlusion
# ===========================================================================================================
# for idx in tqdm(range(num_point)):
#     scan = readScan(scan_path[idx])
#     # Define the conditions
#     angles = np.arctan2(scan[:, 1], scan[:, 0])

#     selected_points1 = scan[(np.radians(30) <= angles) & (angles <= np.radians(90))]
#     selected_points2 = scan[(np.radians(150) <= angles) & (angles <= np.radians(210))]
#     selected_points3 = scan[(np.radians(270) <= angles) & (angles <= np.radians(330))]
#     selected_points = np.concatenate([selected_points1, selected_points2, selected_points3])

#     # selected_points1 = scan[(np.radians(22.5) <= angles) & (angles <= np.radians(67.5))]
#     # selected_points2 = scan[(np.radians(112.5) <= angles) & (angles <= np.radians(157.5))]
#     # selected_points3 = scan[(np.radians(202.5) <= angles) & (angles <= np.radians(247.5))]
#     # selected_points4 = scan[(np.radians(292.5) <= angles) & (angles <= np.radians(337.5))]
#     # print(selected_points1.shape)
#     # print(selected_points2.shape)
#     # print(selected_points3.shape)
#     # selected_points = np.concatenate([selected_points1, selected_points2, selected_points3, selected_points4])
#     selected_points.tofile(save_path+'{}.bin'.format(str(idx).zfill(6)))
# ===========================================================================================================