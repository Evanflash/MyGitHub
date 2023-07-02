cmake_minimum_required(VERSION 3.8)
project(data-publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(include/data-publisher)
ament_export_include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  cv_bridge
)

add_executable(lidar_data_node src/LidarData.cpp)
target_link_libraries(lidar_data_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(lidar_data_node
  ${dependencies}
)

add_executable(radar_data_node src/RadarData.cpp)
target_link_libraries(radar_data_node
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(radar_data_node
  ${dependencies}
)

install(TARGETS
  lidar_data_node
  radar_data_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
