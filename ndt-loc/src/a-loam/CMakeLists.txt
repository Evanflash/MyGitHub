cmake_minimum_required(VERSION 3.8)
project(a-loam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include/a-loam)
ament_export_include_directories(include)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

add_executable(scan_registration_node src/ScanRegistrationNode.cpp src/common.cpp)
target_link_libraries(scan_registration_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(scan_registration_node
  ${dependencies}
)

add_executable(laser_odometry_node src/LaserOdometryNode.cpp src/common.cpp)
target_link_libraries(laser_odometry_node
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
)
ament_target_dependencies(laser_odometry_node
  ${dependencies}
)


install(TARGETS
  scan_registration_node
  laser_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
