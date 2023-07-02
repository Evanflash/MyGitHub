#ifndef _RADAR_DATA_H
#define _RADAR_DATA_H

#include <queue>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

namespace DATA{

class RadarData : public rclcpp::Node{
public:
    RadarData(std::string name, std::string file_path, std::string file_time);

    void Run();
    bool ReadData();
    void ReadTimestamp();
    void publish();

private:
    const std::string radar_data_file_path;
    const std::string radar_data_file_time;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr radarDataPublisher;
    rclcpp::TimerBase::SharedPtr timer;

    cv::Mat curRadarData;
    std::queue<std::string> radar_data_que;

};

} // namespace DATA

#endif // _RADAR_DATA_H