#include "utilty.h"
#include "ImageProjection.h"


namespace ndtloc{

struct pose{
    float dix;
    float diy;
    float yaw;
    long long timestamping;
    pose(std::string _dix, std::string _diy, std::string _yaw, std::string _timestamping){
        dix = std::stof(_dix);
        diy = std::stof(_diy);
        yaw = std::stof(_yaw);
        timestamping = std::stoll(_timestamping);
    }
};

}
using namespace ndtloc;


std::queue<pose> GetPoses(){
    std::queue<pose> poses;
    std::fstream input(ndtloc::radar_timestamping.c_str(), std::ios::in);

    std::string dix, diy, yaw, timestamping, line;
    while(std::getline(input, line)){
        std::stringstream ss(line);
        std::string str;

        std::getline(ss, str, ' ');
        timestamping = str;

        std::getline(ss, str, ' ');
        std::getline(ss, str, ' ');
        dix = str;

        std::getline(ss, str, ' ');
        diy = str;

        std::getline(ss, str, ' ');
        yaw = str;

        poses.push(pose(dix, diy, yaw, timestamping));
    }

    input.close();
    return poses;
}

std::queue<long long> ReadTimestamping(){
    std::queue<long long> lidar_timestamping;
    std::fstream input(ndtloc::_left_lidar_time.c_str(), std::ios::in);
    std::string line;
    while(std::getline(input, line)){
        std::stringstream ss(line);
        std::string str;
        std::getline(ss, str, ' ');
        lidar_timestamping.push(std::stoll(str));
    }
    input.close();
    return lidar_timestamping;
}

void FindNearTimestamping(std::queue<pose>& poses, std::queue<long long>& timestamping){
    long long min_value = LLONG_MAX;
    long long radar_timestamping = poses.front().timestamping;
    while(!timestamping.empty()){
        long long tmp = abs(timestamping.front() - radar_timestamping);
        if(tmp > min_value){
            break;
        }
        min_value = tmp;
        timestamping.pop();
    }
}

Eigen::Affine3f TransMatrix(pose& cur_pose){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << cur_pose.dix, cur_pose.diy, 0.0;
    transform.rotate(Eigen::AngleAxisf(cur_pose.yaw, Eigen::Vector3f::UnitZ()));
    return transform;
}

void ReadPointCloud(const std::string file, CLOUDPTR cloud){
    std::fstream input(file.c_str(), std::ios::in);
    float tmp;
    std::vector<float> data;
    while(!input.eof()){
        input.read((char*)& tmp, sizeof(float));
        data.push_back(tmp);
    }
    input.close();

    size_t size = data.size() / 4;
    
    POINT point;
    for(size_t i = 0; i < size; i++){
        point.x = data[i];
        point.y = data[size + i];
        point.z = data[size + size + i];
        cloud -> push_back(point);
    }
    input.close();
}

void MoveOffset(CLOUDPTR cloud){
    size_t size = cloud -> size();
    for(size_t i = 0; i < size; i++){
        POINT &point = cloud -> points[i];
        point.x = -point.x + _x_offset;
        point.y = -point.y - _y_offset;
        point.z =  point.z + _z_offset;
    }
}

void RunMapping(){
    ImageProjection ip("ImageProjection");

    std::queue<pose> poses = GetPoses();
    std::queue<long long> lidar_data = ReadTimestamping();

    CLOUDPTR global_map_ptr(new CLOUD());
    Eigen::Affine3f pre_pose = Eigen::Affine3f::Identity();
    while(!poses.empty()){
        FindNearTimestamping(poses, lidar_data);
        if(lidar_data.empty()){
            break;
        }
        pose cur_pose = poses.front();
        poses.pop();
        std::string file_name = std::to_string(lidar_data.front());
        lidar_data.pop();
        
        std::cout << "radar :" << cur_pose.timestamping << " " << "lidar : " << file_name << std::endl;

        file_name = _left_lidar_path + "/" + file_name + ".bin";

        CLOUDPTR cur_cloud_ptr(new CLOUD());
        CLOUDPTR trans_cloud_ptr(new CLOUD());

        ReadPointCloud(file_name, cur_cloud_ptr);

        CLOUDPTR filtered_cloud_ptr = ip.Run(cur_cloud_ptr);
        
        MoveOffset(filtered_cloud_ptr);

        Eigen::Affine3f trans = TransMatrix(cur_pose);

        pre_pose = pre_pose * trans;

        
        pcl::transformPointCloud(*filtered_cloud_ptr,
                                 *trans_cloud_ptr,
                                 pre_pose);
        *global_map_ptr += *trans_cloud_ptr;
    }
    CLOUDPTR filtered_map_ptr(new CLOUD());
    pcl::VoxelGrid<POINT> global_map_filter;
    global_map_filter.setLeafSize(0.2, 0.2, 0.2);
    global_map_filter.setInputCloud(global_map_ptr);
    global_map_ptr.reset(new CLOUD());
    global_map_filter.filter(*filtered_map_ptr);

    pcl::io::savePCDFileBinary(map_file_path, *filtered_map_ptr);
    // pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);
    std::cout << "finish mapping!" << std::endl;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    RunMapping();
    rclcpp::shutdown();
    return 0;
}