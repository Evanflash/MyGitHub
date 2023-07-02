#ifndef _A_LOAM_COMMON_H
#define _A_LOAM_COMMON_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

using std::string;

namespace ALOAM{

using PointType = pcl::PointXYZI;
using CloudType = pcl::PointCloud<PointType>;
using CloudTypePtr = CloudType::Ptr;

using SubCloud = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr; 
using PubCloud = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;









extern int N_SCANS;                     // 激光雷达线数
extern float MINMUM_RANGE;
extern double SCAN_PERIOD;
extern double DISTANCE_SQ_THRESHOLD;
extern double NEARBY_SCAN;
extern int DISTORTION;

/**
 * laserOdometry区分队列名
*/
extern string cornerPointsSharpName;
extern string cornerPointsLessSharpName;
extern string surfPointsFlatName;
extern string surfPointsLessFlatName;
extern string laserCloudFullResName;

extern double para_q[4];
extern double para_t[3];

/**
 * laserMapping
*/
extern double parameters[7];

extern string cornerLast;
extern string surfLast;
extern string fullRes;

extern int laserCloudCenWidth;
extern int laserCloudCenHeight;
extern int laserCloudCenDepth;

extern int laserCloudWidth;
extern int laserCloudHeight;
extern int laserCloudDepth;

}



#endif