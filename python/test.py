import argparse

from utils.solid import *
from utils.point_module import *

parser = argparse.ArgumentParser(description= "SOLiD")

## =========================================================================================
##                                           PARAMS
## =========================================================================================
##                 ||   AVIA   || ||   AEVA   || ||   VELODYNE 16   || ||   OUSTER 64   ||
## =========================================================================================
##      Down FOV   ||  -38.6   || ||   -9.6   || ||       -15       || ||    -22.5      ||
##       Up FOV    ||   38.6   || ||    9.6   || ||        15       || ||     22.5      ||
##      Channel    ||    64    || ||    64    || ||        16       || ||      64       ||
##    Max Distance ||    120   || ||    120   || ||        80       || ||      80       ||
## =========================================================================================

## =========================================================================================
##                                   VELODYNE 64 (HDL-64E)
## =========================================================================================
parser.add_argument('--fov_d', type = float, default = '-2',    help = 'FOV')
parser.add_argument('--fov_u', type = float, default =  '24.8', help = 'FOV')
parser.add_argument('--channel', type = int, default =  '64',   help = 'channel')
parser.add_argument('--max_distance', type = int, default = '80', help = 'Far Points Removal')
parser.add_argument('--min_distance', type = int, default = '3',  help = 'Close Points Removal')
# ===============================================================================================

args = parser.parse_args()

# 1. Preprocessing
query_scan = readScan("bin/kitti_00_60/000000.bin")
query_scan = remove_closest_points(query_scan, args.min_distance)
query_scan = remove_far_points(query_scan, args.max_distance)
query_scan = down_sampling(query_scan)

# 2. Generate SOLiD
query_rsolid, query_asolid = get_descriptor(query_scan, args.fov_u, args.fov_d, args.channel, args.max_distance)

# 1. Preprocessing
candidates1_scan = readScan("bin/kitti_00_60/000001.bin")
candidates1_scan = remove_closest_points(candidates1_scan, args.min_distance)
candidates1_scan = remove_far_points(candidates1_scan, args.max_distance)
candidates1_scan = down_sampling(candidates1_scan)

# 2. Generate SOLiD
candidates1_rsolid, candidates1_asolid = get_descriptor(candidates1_scan, args.fov_u, args.fov_d, args.channel, args.max_distance)

# 1. Preprocessing
candidates2_scan = readScan("bin/kitti_00_60/000060.bin")
candidates2_scan = remove_closest_points(candidates2_scan, args.min_distance)
candidates2_scan = remove_far_points(candidates2_scan, args.max_distance)
candidates2_scan = down_sampling(candidates2_scan)

# 2. Generate SOLiD
candidates2_rsolid, candidates2_asolid = get_descriptor(candidates2_scan, args.fov_u, args.fov_d, args.channel, args.max_distance)

similarity1 = np.dot(query_rsolid, candidates1_rsolid) / (np.linalg.norm(query_rsolid) * np.linalg.norm(candidates1_rsolid))
similarity2 = np.dot(query_rsolid, candidates2_rsolid) / (np.linalg.norm(query_rsolid) * np.linalg.norm(candidates2_rsolid))

# 3. Place Recognition
if similarity1 > similarity2:
    initial_cosdist = []
    for shift_index in range(len(candidates1_asolid)):
        initial_cosine_similarity = np.sum(np.abs(query_asolid - np.roll(candidates1_asolid, shift_index)))
        initial_cosdist.append(initial_cosine_similarity)
        initial_angle = (np.argmin(initial_cosdist)) * 6
    print("Initial Heading is : ", initial_angle, " between query and candidates1!!")
else:
    initial_cosdist = []
    for shift_index in range(len(candidates2_asolid)):
        initial_cosine_similarity = np.sum(np.abs(query_asolid - np.roll(candidates2_asolid, shift_index)))
        initial_cosdist.append(initial_cosine_similarity)
        initial_angle = (np.argmin(initial_cosdist)) * 6
    print("Initial Heading is : ", initial_angle, " between query and candidates2!!")