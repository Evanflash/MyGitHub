#include "utilty.h"

namespace ndtloc{

// LidarData config
 std::string _left_lidar_path = "/home/evan/extra/datasets/lidar_data_all/velodyne_left";
 std::string _right_lidar_path = "";
 std::string _left_lidar_time = "/home/evan/extra/datasets/lidar_data_all/left_lidar.txt";
 std::string _right_lidar_time = "/home/evan/extra/datasets/lidar_data_all/no_right.txt";
 float _x_offset = 0.1;
 float _y_offset = 0.47;
 float _z_offset = 0.28;

// front end config
 std::string key_frame_path = "/home/evan/extra/datasets/lidar_data_all/key_frames";
 std::string map_file_path = "/home/evan/extra/datasets/lidar_data_all/map/map.pcd";
 int max_local_map_frames = 20;
 int local_map_filter_frames_num = 10;
 float key_frame_distance = 2.0;

// ImageProjection config
//  std::vector<float> _vertical_angle = {
//     -0.1862, -0.1628, -0.1396, -0.1164, -0.0930, -0.0698, -0.0466, 
//     -0.0232, 0.0, 0.0232, 0.0466, 0.0698, 0.0930, 0.1164, 0.1396, 
//     0.1628, 0.1862, 0.2094, 0.2327, 0.2560, 0.2793, 0.3025, 0.3259, 
//     0.3491, 0.3723, 0.3957, 0.4189, 0.4421, 0.4655, 0.4887, 0.5119, 0.5353};

 int _vertical_scans = 32;
 int _horizontal_scans = 1080;
 float _vertical_ang_start = 0.5353;
 float _vertical_ang_end = -0.1862;
 size_t _segment_valid_num = 30;
 size_t _segment_valid_point_num = 5;
 int _segment_valid_line_num = 3;
 int _ground_scan_index = 20;
 float _segment_theta = 60.0;  


// GroundTruth
 std::string radar_timestamping = "/home/evan/extra/datasets/lidar_data/other-sensors/sub_gt.txt";


}