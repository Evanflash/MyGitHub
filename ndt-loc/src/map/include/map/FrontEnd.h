#ifndef _FRONT_END_H
#define _FRONT_END_H

#include "utilty.h"

namespace ndtloc{

struct Frame{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CLOUDPTR cloud_data;
};

class FrontEnd : public rclcpp::Node{
public:
    FrontEnd(const std::string &name);

    void Update(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);
    void CreateGlobalMap(std_msgs::msg::Bool);

private:
    bool UpdateNewFrame(const Frame& new_key_frame);


private:
    pcl::VoxelGrid<POINT> cloud_filter;
    pcl::VoxelGrid<POINT> local_map_filter;
    pcl::VoxelGrid<POINT> global_map_filter;

    pcl::NormalDistributionsTransform<POINT, POINT>::Ptr ndt_ptr;

    std::deque<Frame> local_map_frames;
    std::deque<Frame> global_map_frames;

    SUBCLOUDPTR sub_point_cloud_in;
    SUBBOOL sub_create_global_map;
    PUBBOOL pub_ready_for_next_cloud;
    PUBBOOL pub_show_map;

    CLOUDPTR local_map_ptr;
    CLOUDPTR global_map_ptr;
    CLOUDPTR result_cloud_ptr;

    Frame current_frame;

    Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
    
};


}

#endif