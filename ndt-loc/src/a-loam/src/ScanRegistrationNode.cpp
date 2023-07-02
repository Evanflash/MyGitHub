#include "ScanRegistration.hpp"

#include <std_msgs/msg/bool.hpp>

namespace ALOAM{
template<typename PointT>
class ScanRegistrationNode : public rclcpp::Node{
    using CLOUD = pcl::PointCloud<PointT>;
public:
    ScanRegistrationNode(string name)
        : Node(name){
        
        RCLCPP_INFO(this -> get_logger(), "ScanRegistrationNode initing");
        RCLCPP_INFO(this -> get_logger(), "-------------------------");

        pubLaserCloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_removed_closed_point", 1);

        pubCornerPointsSharp = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/corner_points_sharp", 1);
        
        pubCornerPointsLessSharp = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/corner_points_less_sharp", 1);

        pubSurfPointsFlat = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/surf_points_flat", 1);

        pubSurfPointsLessFlat = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/surf_points_less_flat", 1);

        pubReadySignal = this -> create_publisher<std_msgs::msg::Bool>(
            "/scan_registration_is_ready", 1);

        subLaserCloudIn = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_ori", 1, std::bind(&ScanRegistrationNode::Run, this, std::placeholders::_1));

        RCLCPP_INFO(this -> get_logger(), "ScanRegistrationNode inited");
    }

    void publish(ScanRegistration<PointT> &sr){
        sensor_msgs::msg::PointCloud2 laserCloudOutMsg;

        pcl::toROSMsg(sr.getLaserCloud(), laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "lidar";
        pubLaserCloud -> publish(laserCloudOutMsg);

        pcl::toROSMsg(sr.getCornerPointsSharp(), laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "lidar";
        pubCornerPointsSharp -> publish(laserCloudOutMsg);

        pcl::toROSMsg(sr.getCornerPointsLessSharp(), laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "lidar";
        pubCornerPointsLessSharp -> publish(laserCloudOutMsg);

        pcl::toROSMsg(sr.getSurfPointsFlat(), laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "lidar";
        pubSurfPointsFlat -> publish(laserCloudOutMsg);

        pcl::toROSMsg(sr.getSurfPointsLessFlat(), laserCloudOutMsg);
        laserCloudOutMsg.header.frame_id = "lidar";
        pubSurfPointsLessFlat -> publish(laserCloudOutMsg);

        std_msgs::msg::Bool isReady;
        isReady.data = true;
        pubReadySignal -> publish(isReady);
    }



    void Run(const sensor_msgs::msg::PointCloud2 &laserCloudMsg){
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(laserCloudMsg, *laserCloudIn);

        ScanRegistration<PointT> sr;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        sr.removeClosedPointCloud(laserCloudIn, laserCloudIn, MINMUM_RANGE);

        sr.extractFeatureFromPointCloud(laserCloudIn);

        publish(sr);
    }


private:
    PubCloud pubLaserCloud;
    PubCloud pubCornerPointsSharp;
    PubCloud pubCornerPointsLessSharp;
    PubCloud pubSurfPointsFlat;
    PubCloud pubSurfPointsLessFlat;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubReadySignal;

    SubCloud subLaserCloudIn;
};



}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<
        ALOAM::ScanRegistrationNode<pcl::PointXYZ>>("scan_registration");
    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    rclcpp::shutdown();

    return 0;
}

