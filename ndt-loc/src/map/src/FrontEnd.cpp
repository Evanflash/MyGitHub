#include "FrontEnd.h"

namespace ndtloc{
FrontEnd::FrontEnd(const std::string &name)
    :Node(name),
    ndt_ptr(new pcl::NormalDistributionsTransform<POINT, POINT>()),
    local_map_ptr(new CLOUD()),
    global_map_ptr(new CLOUD()),
    result_cloud_ptr(new CLOUD()){
    
    RCLCPP_INFO(this -> get_logger(), "create front end node");

    // init param
    sub_point_cloud_in = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
        "/segmented_cloud", 1, std::bind(&FrontEnd::Update, this, std::placeholders::_1));

    sub_create_global_map = this -> create_subscription<std_msgs::msg::Bool>(
        "/create_map", 1, std::bind(&FrontEnd::CreateGlobalMap, this, std::placeholders::_1));
    
    pub_ready_for_next_cloud = this -> create_publisher<std_msgs::msg::Bool>(
        "/read_data", 1);

    pub_show_map = this -> create_publisher<std_msgs::msg::Bool>("/show_map", 1);

    rclcpp::sleep_for(std::chrono::seconds(1));

    cloud_filter.setLeafSize(1.0, 1.0, 1.0);
    local_map_filter.setLeafSize(0.5, 0.5, 0.5);
    global_map_filter.setLeafSize(0.5, 0.2, 0.5);

    ndt_ptr -> setResolution(1.0);
    ndt_ptr -> setStepSize(0.1);
    ndt_ptr -> setTransformationEpsilon(0.01);
    ndt_ptr -> setMaximumIterations(30);

    // publish ready message
    std_msgs::msg::Bool ready_message;
    ready_message.data = true;
    pub_ready_for_next_cloud -> publish(ready_message);
}


void FrontEnd::Update(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg){   
    current_frame.cloud_data.reset(new CLOUD());
    pcl::fromROSMsg(*point_cloud_msg, *current_frame.cloud_data);

    CLOUDPTR filtered_cloud_ptr(new CLOUD());
    cloud_filter.setInputCloud(current_frame.cloud_data);
    cloud_filter.filter(*filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose;
    static Eigen::Matrix4f predict_pose = init_pose;
    static Eigen::Matrix4f last_key_frame_pose = init_pose;

    // init map
    if(local_map_frames.size() == 0){
        current_frame.pose = init_pose;
        UpdateNewFrame(current_frame);

        std_msgs::msg::Bool ready_message;
        ready_message.data = true;
        pub_ready_for_next_cloud -> publish(ready_message);

        return;
    }

    // match ndt
    ndt_ptr -> setInputSource(filtered_cloud_ptr);
    ndt_ptr -> align(*result_cloud_ptr, predict_pose);
    current_frame.pose = ndt_ptr -> getFinalTransformation();

    // update pose
    step_pose = last_pose.inverse() * current_frame.pose;
    predict_pose = current_frame.pose * step_pose;
    last_pose = current_frame.pose;

    // key frame
    if (fabs(last_key_frame_pose(0,3) - current_frame.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame.pose(2,3)) > key_frame_distance) {
        UpdateNewFrame(current_frame);
        last_key_frame_pose = current_frame.pose;
    }

    std_msgs::msg::Bool ready_message;
    ready_message.data = true;
    pub_ready_for_next_cloud -> publish(ready_message);
}

bool FrontEnd::UpdateNewFrame(const Frame& new_key_frame){
    // save key frame
    RCLCPP_INFO(this -> get_logger(), "saving key_frame_%s", std::to_string(global_map_frames.size()).c_str());

    std::string file_path = key_frame_path + "/key_frame_" + std::to_string(global_map_frames.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data);

    // prevent new_key_frame change
    Frame key_frame = new_key_frame;
    key_frame.cloud_data.reset(new CLOUD(*new_key_frame.cloud_data));
    CLOUDPTR transformed_cloud_ptr(new CLOUD());

    // update local map
    local_map_frames.push_back(key_frame);
    while(local_map_frames.size() > max_local_map_frames){
        local_map_frames.pop_front();
    }
    local_map_ptr.reset(new CLOUD());
    for(size_t i = 0; i <local_map_frames.size(); i++){
        pcl::transformPointCloud(*local_map_frames.at(i).cloud_data,
                                 *transformed_cloud_ptr,
                                 local_map_frames.at(i).pose);
        *local_map_ptr += *transformed_cloud_ptr;
    }

    // update target
    if(local_map_frames.size() < local_map_filter_frames_num){
        ndt_ptr -> setInputTarget(local_map_ptr);
    }
    else{
        CLOUDPTR filtered_local_map_ptr(new CLOUD());
        local_map_filter.setInputCloud(local_map_ptr);
        local_map_filter.filter(*filtered_local_map_ptr);
        ndt_ptr -> setInputTarget(filtered_local_map_ptr);
    }

    // update global map
    key_frame.cloud_data.reset(new CLOUD());
    global_map_frames.push_back(key_frame);

    return true;
}

void FrontEnd::CreateGlobalMap(std_msgs::msg::Bool message){
    if(!message.data){
        return;
    }

    local_map_ptr.reset(new CLOUD());
    global_map_ptr.reset(new CLOUD());

    CLOUDPTR key_frame_cloud_ptr(new CLOUD());
    CLOUDPTR transformed_cloud_ptr(new CLOUD());

    for(size_t i = 0; i < global_map_frames.size(); i++){
        std::string file_path = key_frame_path + "/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(file_path, *key_frame_cloud_ptr);

        // do something
        //.....

        // transform
        pcl::transformPointCloud(*key_frame_cloud_ptr,
                                 *transformed_cloud_ptr,
                                 global_map_frames.at(i).pose);

        *global_map_ptr += *transformed_cloud_ptr; 
    }

    CLOUDPTR filtered_map_ptr(new CLOUD());
    global_map_filter.setInputCloud(global_map_ptr);
    global_map_ptr.reset(new CLOUD());
    global_map_filter.filter(*filtered_map_ptr);

    pcl::io::savePCDFileBinary(map_file_path, *filtered_map_ptr);

    RCLCPP_INFO(this -> get_logger(), "finish!");


    // publish show map message
    std_msgs::msg::Bool tmp;
    tmp.data = true;
    pub_show_map -> publish(tmp);
}

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ndtloc::FrontEnd>("front_end_node");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}