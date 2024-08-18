#ifndef SOLiD_H
#define SOLiD_H

#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointType;

inline float calc_dist(float &x, float &y, float &z){return sqrt(x*x + y*y + z*z);}
inline float rad2deg(float rad){return rad * 180.0 / M_PI;}

struct RAH 
{
    int idx_range = 0;
    int idx_angle = 0;
    int idx_height = 0;
};

class SOLiDModule 
{
public:
    Eigen::VectorXd makeSolid(pcl::PointCloud<PointType> & scan_down);

    RAH pt2rah(PointType & scan_down, float gap_angle, float gap_range, float gap_elevation);

    double loop_detection(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate);
    double pose_estimation(const Eigen::VectorXd &query_a, const Eigen::VectorXd &candidate_a);

    void remove_far_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out);
    void remove_closest_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out);
    void down_sampling(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_down);

public:
    const float FOV_u = 2;
    const float FOV_d = -24.8;
    const int NUM_ANGLE = 60;
    const int NUM_RANGE = 40;
    const int NUM_HEIGHT = 32;
    const int MIN_DISTANCE = 3;
    const int MAX_DISTANCE = 80;
    const float VOXEL_SIZE = 0.4;
};

#endif /* SOLiD_H */
