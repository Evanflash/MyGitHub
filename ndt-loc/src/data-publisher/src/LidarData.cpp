#include "LidarData.h"

#include <chrono>


namespace DATA{

LidarData::LidarData(const std::string &name, const std::string &file_path, const std::string &file_time)
    :Node(name), lidar_data_file_path(file_path), 
    lidar_data_file_time(file_time){
    RCLCPP_INFO(this -> get_logger(), "create lidar data node");

    ReadTimestamp();

    pub_laser_data = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
        "/laser_cloud_ori", 1);
    timer = this -> create_wall_timer(std::chrono::milliseconds(500), std::bind(&LidarData::Run, this));
}

void LidarData::Run(){
    // static int nums = 0;
    // nums++;
    // RCLCPP_INFO(this -> get_logger(), "%d", nums);
    if(!lidar_data_que.empty()){
        ReadData();
        PublishCloud();
        lidar_data_que.pop();
    }
}

void LidarData::ReadData(){
    std::string file_path = lidar_data_file_path + "/" + lidar_data_que.front() + ".bin";

    std::fstream input(file_path.c_str(), std::ios::in | std::ios::binary);
    if(input){
        float tmp;
        std::vector<float> data;
        while(!input.eof()){
            input.read((char*)& tmp, sizeof(float));
            data.push_back(tmp);
        }
        input.close();

        size_t size = data.size() / 4;

        _laser_data.reset(new CLOUD());
        _laser_data -> reserve(size);
        POINT point;
        for(size_t i = 0; i < size; i++){
            point.x = data[i];
            point.y = data[size + i];
            point.z = -data[size + size + i];
            _laser_data -> push_back(point);
        }
    }              
}

void LidarData::ReadTimestamp(){
    std::fstream input(lidar_data_file_time.c_str(), std::ios::in);
    if(input){
        std::string line;
        while(std::getline(input, line)){
            std::stringstream ss(line);
            std::string str;

            std::getline(ss, str, ' ');
            lidar_data_que.push(str);
        }
    }
    input.close();
}

void LidarData::PublishCloud(){
    sensor_msgs::msg::PointCloud2 tmp;
    if(pub_laser_data -> get_subscription_count() != 0){
        pcl::toROSMsg(*_laser_data, tmp);
        tmp.header.frame_id = "123";
        pub_laser_data -> publish(tmp);
    }
}

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DATA::LidarData>("lidar_data_node", 
        "/home/evan/extra/datasets/lidar_data_all/velodyne_left", 
        "/home/evan/extra/datasets/lidar_data_all/left_lidar.txt");



    if(rclcpp::ok()){
        rclcpp::spin(node);
    }
    
    rclcpp::shutdown();
    return 0;
}