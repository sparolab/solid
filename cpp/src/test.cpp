#include <solid_module.h>

pcl::PointCloud<PointType>::Ptr read_pcd(const std::string &file_path)
{
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) 
    {
        PCL_ERROR("Couldn't read file\n");
        return nullptr;
    }
    return cloud;
}


int main(int argc, char** argv)
{
    std::vector<Eigen::VectorXd> solid_vector;
    SOLiDModule solid;

    auto query = read_pcd("../pcd/000313.pcd");
    auto candidates1 = read_pcd("../pcd/000314.pcd");
    auto candidates2 = read_pcd("../pcd/000315.pcd");

    // 1. Preprocessing
    pcl::PointCloud<PointType>::Ptr rm_far_points(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr rm_cls_points(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_pts(new pcl::PointCloud<PointType>);
    solid.remove_far_points(*query, rm_far_points);
    solid.remove_closest_points(*rm_far_points, rm_cls_points);
    solid.down_sampling(*rm_cls_points, downsampled_pts);

    // 2. Generate SOLiD
    Eigen::VectorXd query_solid = solid.makeSolid(*downsampled_pts);

    // 1. Preprocessing
    pcl::PointCloud<PointType>::Ptr rm_far_points2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr rm_cls_points2(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_pts2(new pcl::PointCloud<PointType>);
    solid.remove_far_points(*candidates1, rm_far_points2);
    solid.remove_closest_points(*rm_far_points2, rm_cls_points2);
    solid.down_sampling(*rm_cls_points2, downsampled_pts2);

    // 2. Generate SOLiD
    Eigen::VectorXd candidates1_solid = solid.makeSolid(*downsampled_pts2);

    // 1. Preprocessing
    pcl::PointCloud<PointType>::Ptr rm_far_points3(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr rm_cls_points3(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr downsampled_pts3(new pcl::PointCloud<PointType>);
    solid.remove_far_points(*candidates2, rm_far_points3);
    solid.remove_closest_points(*rm_far_points3, rm_cls_points3);
    solid.down_sampling(*rm_cls_points3, downsampled_pts3);    

    // 2. Generate SOLiD
    Eigen::VectorXd cadidates2_solid = solid.makeSolid(*downsampled_pts3);

    // 3. Place Recognition
    double similarity1 = solid.loop_detection(query_solid, candidates1_solid);
    double similarity2 = solid.loop_detection(query_solid, cadidates2_solid);

    double initial_angle;
    if (similarity1 > similarity2) {
        std::cout << "Candidates 1 is decided a revisited place!! " << std::endl;
        initial_angle = solid.pose_estimation(query_solid, candidates1_solid);
        std::cout << "Initial Heading is : " << initial_angle << "between query and candidates1!!" << std::endl;
    }
    else {
        std::cout << "Candidates 2 is decided a revisited place!! " << std::endl;
        initial_angle = solid.pose_estimation(query_solid, cadidates2_solid);
        std::cout << "Initial Heading is : " << initial_angle << "between query and candidates1!!" << std::endl;
    }
}