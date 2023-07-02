#include "RadarFeatures.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


namespace radar{

class RadarFeaturesNode : public rclcpp::Node{
public:
    RadarFeaturesNode(std::string name)
        : Node(name)
    {
        subRadarImage = this -> create_subscription<sensor_msgs::msg::Image>(
            "/radar_data_ori", 1, std::bind(&RadarFeaturesNode::radarImageHandler, this, std::placeholders::_1));
        pubRadarCloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
            "/radar_cloud", 1);
    }

    void radarImageHandler(const sensor_msgs::msg::Image &radarImage){
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(radarImage, sensor_msgs::image_encodings::MONO8);
        cv::Mat radarDataWithInfo = cv_ptr -> image;
        
        // 将radar image中的数据部分提取出来处理
        const int str = 11;
        const int cols = radarDataWithInfo.cols;
        cv::Mat fft_data = cv::Mat::zeros(radarDataWithInfo.rows, cols - str, CV_32F);
        // 角度
        std::vector<float> azimuth(radarDataWithInfo.rows);
        // 角度解码
        static const float encode = M_PI / 2800.0;
        for(int i = 0; i < fft_data.rows; ++i){
            uint16_t lowByte = (uint16_t)radarDataWithInfo.at<uchar>(i, 9);
            uint16_t highByte = (uint16_t)radarDataWithInfo.at<uchar>(i, 8);
            azimuth[i] = (float)((highByte << 8) + lowByte);
            azimuth[i] *= encode;
            for(int j = 0; j < fft_data.cols; ++j){
                fft_data.at<float>(i, j) = (float)radarDataWithInfo.at<uchar>(i, str + j) / 255;
            }
        }

        // 提取特征
        RadarFeatures rf;
        rf.Cen2019Features(fft_data);
        std::vector<std::pair<int, int>> keyPoints = rf.getKeyPointsCoor();

        // 转换为radar point cloud
        static const float reso = 0.0438; // 分辨率
        pcl::PointCloud<pcl::PointXYZI>::Ptr radar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        radar_cloud -> resize(keyPoints.size());
        
        for(uint i = 0; i < keyPoints.size(); ++i){
            pcl::PointXYZI point;
            float dis = (keyPoints[i].second + 1) * reso;
            point.x = dis * cos(azimuth[keyPoints[i].first]);
            point.y = dis * sin(azimuth[keyPoints[i].first]);
            point.z = 0.0;
            point.intensity = fft_data.at<float>(keyPoints[i].first + 11, keyPoints[i].second);
            radar_cloud -> points[i] = point;
        }

        // 发布消息
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*radar_cloud, msg);
        msg.header.frame_id = "radar";
        pubRadarCloud -> publish(msg);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subRadarImage;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRadarCloud;
};

} // namespace radar

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar::RadarFeaturesNode>("radar_feature_node");
    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}