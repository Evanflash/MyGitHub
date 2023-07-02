#ifndef _LIDARDATA_H
#define _LIDARDATA_H


#include <string>
#include <queue>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace DATA{

using POINT = pcl::PointXYZ;
using CLOUD = pcl::PointCloud<POINT>;
using CLOUDPTR = CLOUD::Ptr;
using SUBCLOUDPTR = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr; 
using PUBCLOUDPTR = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

class LidarData : public rclcpp::Node{
public:
    LidarData(const std::string &name, const std::string &file_path, const std::string &file_time);
    void Run();

private:
    void ReadData();
    void ReadTimestamp();
    void PublishCloud();

private:
    const std::string lidar_data_file_path;
    const std::string lidar_data_file_time;

    CLOUDPTR _laser_data;

    PUBCLOUDPTR pub_laser_data;

    std::queue<std::string> lidar_data_que;
    rclcpp::TimerBase::SharedPtr timer;
};

}

#endif