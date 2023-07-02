#include "ShowMap.h"

namespace ndtloc{

void ShowMap::Show(){
    CLOUDPTR map_cloud(new CLOUD());
    pcl::io::loadPCDFile(map_file_path, *map_cloud);

    pcl::visualization::PCLVisualizer viewer("map");
    viewer.addPointCloud(map_cloud);
    viewer.spin();

}

}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    

    ndtloc::ShowMap show_map;
    show_map.Show();


    return 0;
}