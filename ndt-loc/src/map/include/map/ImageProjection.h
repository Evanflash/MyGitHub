#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utilty.h"

namespace ndtloc{

using POINTV = pcl::PointXYZI;
using CLOUDV = pcl::PointCloud<POINTV>;
using CLOUDVPTR = CLOUDV::Ptr;
using Coord2D = Eigen::Vector2i;


class ImageProjection : public rclcpp::Node{
public:
    ImageProjection(const std::string &name);

    void CloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);
    void ProjectPointCloud();
    void GroundRemoval();
    void CloudSegmentation();                       
    void LabelComponents(int row, int col);        
    void PublishCloud();
    void ResetParameters();
    void SetLaserCloudIn(CLOUDPTR cloud);
    CLOUDPTR GetSegmentCloud();
    CLOUDPTR Run(CLOUDPTR cloud);

private:
    CLOUDVPTR laser_cloud_in;

    CLOUDVPTR full_cloud;

    CLOUDVPTR ground_cloud;
    CLOUDVPTR segmented_cloud;

    SUBCLOUDPTR sub_laser_cloud_in;
    PUBCLOUDPTR pub_ground_cloud;
    PUBCLOUDPTR pub_segmented_cloud;

    Eigen::MatrixXf range_mat;
    Eigen::MatrixXf ground_mat;
    Eigen::MatrixXf label_mat;

    int label_count;

    float _ang_resolution_X;
    float _ang_resolution_Y;
    float ang_bottom;
    float segment_theta;

    PUBBOOL pub_ready_for_next_cloud;
};

}
#endif