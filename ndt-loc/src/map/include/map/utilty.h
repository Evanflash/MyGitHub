#ifndef _UTILTY_NDT_LOC_H
#define _UTILTY_NDT_LOC_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>
#include <Eigen/Dense>
#include <limits.h>
#include <string>
#include <algorithm>
#include <fstream>
#include <deque>
#include <queue>
#include <functional>


namespace ndtloc{

using POINT = pcl::PointXYZ;
using CLOUD = pcl::PointCloud<POINT>;
using CLOUDPTR = CLOUD::Ptr;
using SUBCLOUDPTR = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr; 
using PUBCLOUDPTR = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
using SUBBOOL = rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr;
using PUBBOOL = rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr;



// LidarData config
extern std::string _left_lidar_path;
extern std::string _right_lidar_path;
extern std::string _left_lidar_time;
extern std::string _right_lidar_time;
extern float _x_offset;
extern float _y_offset;
extern float _z_offset;

// front end config
extern std::string key_frame_path;
extern std::string map_file_path;
extern int max_local_map_frames;
extern int local_map_filter_frames_num;
extern float key_frame_distance;

// ImageProjection config
// extern std::vector<float> _vertical_angle = {
//     -0.1862, -0.1628, -0.1396, -0.1164, -0.0930, -0.0698, -0.0466, 
//     -0.0232, 0.0, 0.0232, 0.0466, 0.0698, 0.0930, 0.1164, 0.1396, 
//     0.1628, 0.1862, 0.2094, 0.2327, 0.2560, 0.2793, 0.3025, 0.3259, 
//     0.3491, 0.3723, 0.3957, 0.4189, 0.4421, 0.4655, 0.4887, 0.5119, 0.5353};

extern int _vertical_scans;
extern int _horizontal_scans;
extern float _vertical_ang_start;
extern float _vertical_ang_end;
extern size_t _segment_valid_num;
extern size_t _segment_valid_point_num;
extern int _segment_valid_line_num;
extern int _ground_scan_index;
extern float _segment_theta;  


// GroundTruth
extern std::string radar_timestamping;



}

#endif