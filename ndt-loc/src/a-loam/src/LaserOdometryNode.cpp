#include "LaserOdometry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ALOAM{

class LaserOdometryNode : public rclcpp::Node {
public:
    LaserOdometryNode(std::string name)
        : Node(name){
        pubLaserOdom = this -> create_publisher<nav_msgs::msg::Odometry>(
            "/lidar_init_odom", 1);
        pubLaserPath = this -> create_publisher<nav_msgs::msg::Path>(
            "/lidar_path", 1);

        subLaserPointsCornerSharp = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/corner_points_sharp", 1, std::bind(&LaserOdometryNode::updateCornerSharp, this, std::placeholders::_1));
        subLaserPointsCornerLessSharp = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/corner_points_less_sharp", 1, std::bind(&LaserOdometryNode::updateCornerLessSharp, this, std::placeholders::_1));
        subLaserPointsSurfFlat = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/surf_points_flat", 1, std::bind(&LaserOdometryNode::updateSurfFlat, this, std::placeholders::_1));
        subLaserPointsSurfLessFlat = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/surf_points_less_flat", 1, std::bind(&LaserOdometryNode::updateSurfLessFlat, this, std::placeholders::_1));
        subLaserPointsFullRes = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
            "/laser_cloud_removed_closed_point", 1, std::bind(&LaserOdometryNode::updateFullRes, this, std::placeholders::_1));

        subReadyToRun = this -> create_subscription<std_msgs::msg::Bool>(
            "/scan_registration_is_ready", 1, std::bind(&LaserOdometryNode::Run, this, std::placeholders::_1));
    }

    /**
     * 更新队列
    */
    void updateCornerSharp(const sensor_msgs::msg::PointCloud2 &cloud){
        CloudTypePtr cloud_out(new CloudType());
        pcl::fromROSMsg(cloud, *cloud_out);
        lo.queueHandler(cloud_out, cornerPointsSharpName);
    }
    void updateCornerLessSharp(const sensor_msgs::msg::PointCloud2 &cloud){
        CloudTypePtr cloud_out(new CloudType());
        pcl::fromROSMsg(cloud, *cloud_out);
        lo.queueHandler(cloud_out, cornerPointsLessSharpName);
    }
    void updateSurfFlat(const sensor_msgs::msg::PointCloud2 &cloud){
        CloudTypePtr cloud_out(new CloudType());
        pcl::fromROSMsg(cloud, *cloud_out);
        lo.queueHandler(cloud_out, surfPointsFlatName);
    }
    void updateSurfLessFlat(const sensor_msgs::msg::PointCloud2 &cloud){
        CloudTypePtr cloud_out(new CloudType());
        pcl::fromROSMsg(cloud, *cloud_out);
        lo.queueHandler(cloud_out, surfPointsLessFlatName);
    }
    void updateFullRes(const sensor_msgs::msg::PointCloud2 &cloud){
        CloudTypePtr cloud_out(new CloudType());
        pcl::fromROSMsg(cloud, *cloud_out);
        lo.queueHandler(cloud_out, laserCloudFullResName);
    }
    
    /**
     * 运行
    */
    void Run(const std_msgs::msg::Bool &ready){
        if(!ready.data){
            return;
        }
        lo.odometry();
        publishOdometry();
    }

    /**
     * 发布消息
    */
    void publishOdometry(){
        nav_msgs::msg::Odometry laserOdometry;
        Eigen::Quaterniond q = lo.getCurrQuateriniond();
        Eigen::Vector3d t = lo.getCurrVector();

        static int nums = 0;
        nums++;

        // RCLCPP_INFO(this -> get_logger(), 
            // "%f, %f, %f, %d", t.x(), t.y(), t.z(), nums);
        

        laserOdometry.header.frame_id = "lidar";
        laserOdometry.child_frame_id = "/lidar_odom";
        laserOdometry.pose.pose.orientation.x = q.x();
        laserOdometry.pose.pose.orientation.y = q.y();
        laserOdometry.pose.pose.orientation.z = q.z();
        laserOdometry.pose.pose.orientation.w = q.w();
        laserOdometry.pose.pose.position.x = t.x();
        laserOdometry.pose.pose.position.y = t.y();
        laserOdometry.pose.pose.position.z = t.z();
        pubLaserOdom -> publish(laserOdometry);

        geometry_msgs::msg::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;

        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "lidar";
        
        pubLaserPath -> publish(laserPath);
    }


private:
    LaserOdometry lo;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdom;
    nav_msgs::msg::Path laserPath;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath;

    SubCloud subLaserPointsCornerSharp;
    SubCloud subLaserPointsCornerLessSharp;
    SubCloud subLaserPointsSurfFlat;
    SubCloud subLaserPointsSurfLessFlat;
    SubCloud subLaserPointsFullRes;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subReadyToRun;
};

} // namespace ALOAM

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ALOAM::LaserOdometryNode>("laser_odometry");
    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    rclcpp::shutdown();

    return 0;
}
