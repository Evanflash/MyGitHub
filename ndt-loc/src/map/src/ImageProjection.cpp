#include <boost/circular_buffer.hpp>
#include "ImageProjection.h"

namespace ndtloc{
ImageProjection::ImageProjection(const std::string &name)
    : Node(name){
    sub_laser_cloud_in = this -> create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", 1, std::bind(&ImageProjection::CloudHandler, this, std::placeholders::_1));

    pub_ground_cloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
        "/ground_cloud", 1);
    
    pub_segmented_cloud = this -> create_publisher<sensor_msgs::msg::PointCloud2>(
        "/segmented_cloud", 1);

    pub_ready_for_next_cloud = this -> create_publisher<std_msgs::msg::Bool>("/read_data", 1);

    _ang_resolution_X = (M_PI * 2) / _horizontal_scans;
    _ang_resolution_Y = -(_vertical_ang_end - _vertical_ang_start) / float(_vertical_scans - 1);
    ang_bottom = _vertical_ang_start + 0.01;
    segment_theta = _segment_theta / 180.0 * M_PI;

    // publish ready message
    // std_msgs::msg::Bool ready_message;
    // ready_message.data = true;
    // pub_ready_for_next_cloud -> publish(ready_message);
}

void ImageProjection::ResetParameters(){
    laser_cloud_in.reset(new CLOUDV());
    full_cloud.reset(new CLOUDV());
    ground_cloud.reset(new CLOUDV());
    segmented_cloud.reset(new CLOUDV());

    range_mat.resize(_vertical_scans, _horizontal_scans);
    ground_mat.resize(_vertical_scans, _horizontal_scans);
    label_mat.resize(_vertical_scans, _horizontal_scans);

    range_mat.fill(FLT_MAX);
    ground_mat.setZero();
    label_mat.setZero();

    label_count = 1;

    const size_t cloud_size = _vertical_scans * _horizontal_scans;
    const POINTV point(0.0, 0.0, 0.0, -1);
    full_cloud -> points.resize(cloud_size);
    std::fill(full_cloud -> points.begin(), full_cloud -> points.end(), point);
}

void ImageProjection::CloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg){
    ResetParameters();

    CLOUDPTR point_cloud(new CLOUD());
    pcl::fromROSMsg(*laserCloudMsg, *point_cloud);
    laser_cloud_in -> points.resize(point_cloud -> size());
    for(size_t i = 0; i < point_cloud -> size(); i++){
        POINT point = point_cloud -> points[i];
        POINTV pointv(point.x, point.y, point.z, 0);
        laser_cloud_in -> points[i] = pointv; 
    }
    point_cloud.reset(new CLOUD());

    ProjectPointCloud();
    GroundRemoval(); 
    CloudSegmentation();
    PublishCloud();
}

void ImageProjection::ProjectPointCloud(){
    const size_t cloudSize = laser_cloud_in -> points.size();
    
    for(size_t i = 0; i < cloudSize; i++){
        POINTV thisPoint = laser_cloud_in -> points[i];

        float range = sqrt(thisPoint.x * thisPoint.x +
                           thisPoint.y * thisPoint.y +
                           thisPoint.z * thisPoint.z);
        float verticalAngle = std::asin(thisPoint.z / range);

        // remove close points
        if(range < 3.0){
            continue;
        }

        int rowInd = (-verticalAngle + ang_bottom) / _ang_resolution_Y;
        if(rowInd < 0 || rowInd >= _vertical_scans){
            continue;
        }

        float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);
        int colInd = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

        if(colInd >= _horizontal_scans){
            colInd -= _horizontal_scans;
        }

        if(colInd < 0 || colInd >= _horizontal_scans){
            continue;
        }

        range_mat(rowInd, colInd) = range;
        
        thisPoint.intensity = 0;
        size_t index = colInd + rowInd * _horizontal_scans;
        full_cloud -> points[index] = thisPoint;
    }
}

void ImageProjection::GroundRemoval(){
    // ground_mat
    // -1, cant check if ground or not
    // 0, not ground
    // 1, ground
   for(int j = 0; j < _horizontal_scans; j++){
        for(int i = 0; i < _ground_scan_index; i++){
            int lowerInd = j + i * _horizontal_scans;
            int upperInd = lowerInd + _horizontal_scans;
            // std::cout << 111 << std::endl;
            if(full_cloud -> points[lowerInd].intensity == -1 ||
               full_cloud -> points[upperInd].intensity == -1){
                ground_mat(i, j) = -1;
                continue;
            }

            float dX =
                full_cloud->points[upperInd].x - full_cloud->points[lowerInd].x;
            float dY =
                full_cloud->points[upperInd].y - full_cloud->points[lowerInd].y;
            float dZ =
                full_cloud->points[upperInd].z - full_cloud->points[lowerInd].z;

            float vertical_angle = std::atan2(abs(dZ) , sqrt(dX * dX + dY * dY + dZ * dZ));

            if(vertical_angle <= 10.0 / 180.0 * M_PI){
                ground_mat(i, j) = 1;
                ground_mat(i + 1, j) = 1;
            }
        }
    }

    // mark these points that dont need for segmentation
    for(int i = 0; i < _vertical_scans; i++){
        for(int j = 0; j < _horizontal_scans; j++){
            if(ground_mat(i, j) == 1 || range_mat(i, j) == FLT_MAX){
                label_mat(i, j) = -1;
            }
        }
    }

    for(int i = 0; i < _ground_scan_index; i++){
        for(int j = 0; j < _horizontal_scans; j++){
            if(ground_mat(i, j) == 1){
                ground_cloud -> push_back(full_cloud -> points[j + i * _horizontal_scans]);
            }
        }
    }

}

void ImageProjection::CloudSegmentation(){
    // label the points
    for(int i = 0; i < _vertical_scans; i++){
        for(int j = 0; j < _horizontal_scans; j++){
            if(label_mat(i, j) == 0)
                LabelComponents(i, j);
        }
    }

    // extract segmented cloud
    for(int i = 0; i < _vertical_scans; i++){
        for(int j = 0; j < _horizontal_scans; j++){
            if(label_mat(i, j) > 0 && label_mat(i, j) != 999999){
                segmented_cloud -> push_back(
                    full_cloud -> points[j + i * _horizontal_scans]);
                
                segmented_cloud -> points.back().intensity = label_mat(i, j);
            }

        }
    }
}  


void ImageProjection::LabelComponents(int row, int col){
    const float segmentThetaThreshold = tan(_segment_theta);
    
    std::vector<bool> lineCountFlag(_vertical_scans, false);
    const size_t cloud_size = _vertical_scans * _horizontal_scans;
    boost::circular_buffer<Coord2D> queue(cloud_size);
    boost::circular_buffer<Coord2D> all_pushed(cloud_size);

    queue.push_back({row, col});
    all_pushed.push_back({row, col});

    const Coord2D neighborIterator[4] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

    while(queue.size() > 0){
        // pop point, get one point
        Coord2D fromInd = queue.front();
        queue.pop_front();

        // give this point a label
        label_mat(fromInd.x(), fromInd.y()) = label_count;

        // loop through all the neighboring grids of poped grid
        for(const auto &iter : neighborIterator){
            // new index
            int thisIndX = fromInd.x() + iter.x();
            int thisIndY = fromInd.y() + iter.y();

            // out of range
            if(thisIndX < 0 || thisIndX >= _vertical_scans){
                continue;
            }

            thisIndY = (thisIndY + _horizontal_scans) % _horizontal_scans;

            if(label_mat(thisIndX, thisIndY) != 0){
                continue;
            }

            float d1 = std::max(range_mat(fromInd.x(), fromInd.y()),
                                range_mat(thisIndX, thisIndY));
            float d2 = std::min(range_mat(fromInd.x(), fromInd.y()),
                                range_mat(thisIndX, thisIndY));

            float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
            float tang = (d2 * sin(alpha)) / (d1 - d2 * cos(alpha));

            if(tang > segmentThetaThreshold){
                queue.push_back({thisIndX, thisIndY});

                label_mat(thisIndX, thisIndY) = label_count;
                lineCountFlag[thisIndX] = true;

                all_pushed.push_back({thisIndX, thisIndY});
            }
        }
    }

    // check if this segment is valid
    bool feasibleSegment = false;
    if(all_pushed.size() > _segment_valid_num){
        feasibleSegment = true;
    }
    else if(all_pushed.size() >= _segment_valid_point_num){
        int lineCount = 0;
        for(int i = 0; i < _vertical_scans; i++){
            if(lineCountFlag[i] == true) ++lineCount;
        }
        if(lineCount >= _segment_valid_line_num) feasibleSegment = true;
    }

    if(feasibleSegment == true){
        ++label_count;
    }
    else{
        for(size_t i = 0; i < all_pushed.size(); i++){
            label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
        }
    }
    
}



void ImageProjection::PublishCloud(){
    CLOUDPTR points(new CLOUD());
    for(size_t i = 0; i < segmented_cloud -> points.size(); i++){
        POINTV pointv = segmented_cloud -> points[i];
        POINT point(pointv.x, pointv.y, pointv.z);
        points -> push_back(point);
    }

    sensor_msgs::msg::PointCloud2 tmp_cloud;
    pcl::toROSMsg(*points, tmp_cloud);
    pub_segmented_cloud -> publish(tmp_cloud);
}

void ImageProjection::SetLaserCloudIn(CLOUDPTR point_cloud){
    laser_cloud_in.reset(new CLOUDV());
    for(size_t i = 0; i < point_cloud -> size(); i++){
        POINT point = point_cloud -> points[i];
        POINTV pointv(point.x, point.y, point.z, 0);
        laser_cloud_in -> push_back(pointv); 
    }
}

CLOUDPTR ImageProjection::GetSegmentCloud(){
    CLOUDPTR points(new CLOUD());
    for(size_t i = 0; i < segmented_cloud -> points.size(); i++){
        POINTV pointv = segmented_cloud -> points[i];
        POINT point(pointv.x, pointv.y, pointv.z);
        points -> push_back(point);
    }
    return points;
}

CLOUDPTR ImageProjection::Run(CLOUDPTR cloud){

    ResetParameters();
 
    SetLaserCloudIn(cloud);

    ProjectPointCloud();
 
    GroundRemoval(); 

    CloudSegmentation();

    return GetSegmentCloud();
}

}
// int main(int argc, char** argv){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ndtloc::ImageProjection>("image_projection");
//     rclcpp::spin(node);
//     rclcpp::shutdown();

//     return 0;
// }