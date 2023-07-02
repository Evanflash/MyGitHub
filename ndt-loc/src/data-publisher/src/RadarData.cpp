#include "RadarData.h"

#include <fstream>
#include <chrono>

#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

namespace DATA{
RadarData::RadarData(std::string name, std::string file_path, std::string file_time)
    : Node(name),
      radar_data_file_path(file_path),
      radar_data_file_time(file_time){
    radarDataPublisher = this -> create_publisher<sensor_msgs::msg::Image>(
        "/radar_data_ori", 1);
    timer = this -> create_wall_timer(std::chrono::milliseconds(500), std::bind(&RadarData::Run, this));

    ReadTimestamp();
}

void RadarData::Run(){
    if(ReadData()){
        publish();
    }
}

bool RadarData::ReadData(){
    if(!radar_data_que.empty()){
        std::string file = radar_data_file_path + "/" + radar_data_que.front() + ".png";
        radar_data_que.pop();

        curRadarData = cv::imread(file, 0);
        return true;
    }
    return false;
}

void RadarData::ReadTimestamp(){
    std::fstream input(radar_data_file_time.c_str(), std::ios::in);
    if(input){
        std::string line;
        while(std::getline(input, line)){
            std::stringstream ss(line);
            std::string str;

            std::getline(ss, str, ' ');
            radar_data_que.push(str);
        }
    }
    input.close();
}

void RadarData::publish(){
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
        std_msgs::msg::Header(), "mono8", curRadarData).toImageMsg();
    radarDataPublisher -> publish(*msg);
}

}// namespace DATA

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DATA::RadarData>("radar_data_node", 
        "/home/evan/extra/datasets/radar_example/radar", 
        "/home/evan/extra/datasets/radar_example/radar.timestamps");



    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}