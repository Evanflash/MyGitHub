cmake_minimum_required(VERSION 3.8)
project(map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)


include_directories(include/map)
ament_export_include_directories(include)


set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
)


add_library(mapping_image
  src/ImageProjection.cpp
  src/utilty.cpp
)
target_link_libraries(mapping_image
  ${PCL_LIBRARIES}
)
ament_target_dependencies(mapping_image
  ${dependencies}
)




add_executable(lidar_data_node src/LidarData.cpp src/utilty.cpp)
target_link_libraries(lidar_data_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(lidar_data_node
  ${dependencies}
  Eigen3
)

# add_executable(image_projection_node src/ImageProjection.cpp src/utilty.cpp)
# target_link_libraries(image_projection_node
#   ${PCL_LIBRARIES}
# )
# ament_target_dependencies(image_projection_node
#   ${dependencies}
# )

add_executable(front_end_node src/FrontEnd.cpp src/utilty.cpp)
target_link_libraries(front_end_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(front_end_node
  ${dependencies}
)

add_executable(show_map_node src/ShowMap.cpp src/utilty.cpp)
target_link_libraries(show_map_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(show_map_node
  ${dependencies}
)

add_executable(mapping_node src/GroundTruth.cpp src/utilty.cpp)
target_link_libraries(mapping_node
  mapping_image
  ${PCL_LIBRARIES}
)
ament_target_dependencies(mapping_node
  ${dependencies}
)

install(TARGETS
  lidar_data_node
  # image_projection_node
  front_end_node
  show_map_node
  mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
