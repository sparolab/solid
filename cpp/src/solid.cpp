#include <solid_module.h>

////////////////////////////// solid module //////////////////////////////
float xy2theta(float &x, float &y)
{
    if (x >= 0 && y >= 0)
        return (180/M_PI) * atan(y/x);
    if (x < 0 && y > 0)
        return 180 - ((180/M_PI) ) * atan(y/(-x));
    if (x < 0 && y < 0)
        return 180 + ((180/M_PI) ) * atan(y/x);
    if (x >= 0 && y < 0)
        return 360 - ((180/M_PI) * atan((-y)/x));
}

RAH SOLiDModule::pt2rah(PointType & point, float gap_angle, float gap_range, float gap_height)
{
    RAH rah;
    float point_x = point.x;
    float point_y = point.y;
    float point_z = point.z;

    if(point_x == 0.0)
        point_x = 0.001;
    if(point_y == 0.0)
        point_y = 0.001;

    float theta = xy2theta(point_x, point_y);
    float dist_xy = sqrt(point_x*point_x + point_y*point_y);
    float phi = rad2deg(atan2(point_z, dist_xy));
    rah.idx_range = std::min(static_cast<int>(dist_xy / gap_range), NUM_RANGE - 1);
    rah.idx_angle = std::min(static_cast<int>(theta / gap_angle), NUM_ANGLE - 1);
    rah.idx_height = std::min(static_cast<int>((phi - FOV_d)/gap_height), NUM_HEIGHT - 1);
    // std::cout << rah.idx_height << std::endl;
    return rah;
}

Eigen::VectorXd SOLiDModule::makeSolid(pcl::PointCloud<PointType> &scan_down)
{   
    Eigen::MatrixXd range_matrix(NUM_RANGE, NUM_HEIGHT);
    Eigen::MatrixXd angle_matrix(NUM_ANGLE, NUM_HEIGHT);
    Eigen::VectorXd solid(NUM_RANGE  + NUM_ANGLE);
    range_matrix.setZero();
    angle_matrix.setZero();
    solid.setZero();

    float gap_angle = 360.0/NUM_ANGLE;
    float gap_range = static_cast<float>(MAX_DISTANCE)/NUM_RANGE;
    float gap_height = (FOV_u - FOV_d) / NUM_HEIGHT;
    
    for(int i = 0; i < scan_down.points.size(); i++)
    {
        RAH rah = pt2rah(scan_down.points[i], gap_angle, gap_range, gap_height);
        range_matrix(rah.idx_range, rah.idx_height) +=1;
        angle_matrix(rah.idx_angle, rah.idx_height) +=1;
    }

    Eigen::VectorXd number_vector(NUM_HEIGHT);
    number_vector.setZero();
    for(int col_idx=0; col_idx<range_matrix.cols(); col_idx++)
    {
        number_vector(col_idx) = range_matrix.col(col_idx).sum();
    }

    double min_val = number_vector.minCoeff();
    double max_val = number_vector.maxCoeff();
    number_vector = (number_vector.array() - min_val) / (max_val - min_val);

    Eigen::VectorXd range_solid = range_matrix * number_vector;
    Eigen::VectorXd angle_solid = angle_matrix * number_vector;

    solid.head(NUM_RANGE) = range_solid;
    solid.tail(NUM_ANGLE) = angle_solid;
    return solid;
}

double SOLiDModule::loop_detection(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate)
{
    Eigen::VectorXd r_query = query.segment(0, NUM_RANGE);
    Eigen::VectorXd r_candidate = candidate.segment(0, NUM_RANGE);
    double cosine_similarity = (r_query.dot(r_candidate))/(r_query.norm() * r_candidate.norm());
    return cosine_similarity;
}

double SOLiDModule::pose_estimation(const Eigen::VectorXd &query, const Eigen::VectorXd &candidate)
{
    Eigen::VectorXd a_query = query.segment(NUM_RANGE, NUM_ANGLE);
    Eigen::VectorXd a_candidate = candidate.segment(NUM_RANGE, NUM_ANGLE);

    double minL1normDist = std::numeric_limits<double>::max();
    int minIndex = 0;
    int numAngle = a_query.size();
    
    for(int shiftIndex = 0; shiftIndex < numAngle; ++shiftIndex)
    {
        Eigen::VectorXd shiftedQuery = Eigen::VectorXd::Zero(numAngle);
        for (int i = 0; i < numAngle; ++i)
        {
            shiftedQuery((i+shiftIndex) % numAngle) = a_query(i);
        }
        double L1NormDist = (a_candidate - shiftedQuery).cwiseAbs().sum();
        if(L1NormDist < minL1normDist)
        {
            minL1normDist = L1NormDist;
            minIndex = shiftIndex;
        }
    }
    double angleDifference = (minIndex + 1) * (360.0 / numAngle);
    return angleDifference;
}
//////////////////////////////////////////////////////////////////////////

////////////////////////////// point module //////////////////////////////
void SOLiDModule::remove_far_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out)
{
    for(int i = 0; i < scan_raw.points.size(); i++)
    {
        float dist = calc_dist(scan_raw.points[i].x, scan_raw.points[i].y, scan_raw.points[i].z);
        if(dist < MAX_DISTANCE)
        {
            scan_out->points.push_back(scan_raw.points[i]);
        }
    }
}

void SOLiDModule::remove_closest_points(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_out)
{
    for(int i = 0; i < scan_raw.size(); i++)
    {
        float dist = calc_dist(scan_raw.points[i].x, scan_raw.points[i].y, scan_raw.points[i].z);
        if(dist > MIN_DISTANCE)
        {
            scan_out->points.push_back(scan_raw.points[i]);
        }
    }
}

void SOLiDModule::down_sampling(pcl::PointCloud<PointType> & scan_raw, pcl::PointCloud<PointType>::Ptr scan_down)
{
    pcl::PointCloud<PointType> Data_for_Voxel;
    pcl::VoxelGrid<PointType> downSizeFilterSolid;

    copyPointCloud(scan_raw, Data_for_Voxel);
    downSizeFilterSolid.setInputCloud(Data_for_Voxel.makeShared());
    downSizeFilterSolid.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    downSizeFilterSolid.filter(*scan_down);
}