cmake_minimum_required(VERSION 3.8)
project(radar-odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)

include_directories(include/radar-odometry)
ament_export_include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  cv_bridge
)

add_executable(radar_feature_node src/RadarFeaturesNode.cpp)
target_link_libraries(radar_feature_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(radar_feature_node ${dependencies})

install(TARGETS
  radar_feature_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
