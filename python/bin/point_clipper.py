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

bin_path = '/input/bin/'
save_path = '/save/bin/'    
os.makedirs(save_path, exist_ok=True)

scan_path, num_point = point_path(bin_path)

# Point Clipping
# ===========================================================================================================
angle_threshold1 = np.radians(degree = None)
angle_threshold2 = np.radians(degree = None)  

for idx in tqdm(range(num_point)):
    scan = readScan(scan_path[idx])
    angles = np.arctan2(scan[:, 1], scan[:, 0])
    selected_points = scan[((angles >= angle_threshold1) & (angles <= angle_threshold2))]
    selected_points.tofile(save_path+'{}.bin'.format(str(idx).zfill(6)))
# ===========================================================================================================
