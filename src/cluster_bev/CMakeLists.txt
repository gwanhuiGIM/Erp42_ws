cmake_minimum_required(VERSION 3.8)
project(cluster_bev)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(pcl_conversions REQUIRED)

add_executable(cluster_bev_node src/cluster_bev_node.cpp)

ament_target_dependencies(cluster_bev_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

target_include_directories(cluster_bev_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(cluster_bev_node
  ${PCL_LIBRARIES}
)

target_compile_definitions(cluster_bev_node PRIVATE
  ${PCL_DEFINITIONS}
)

install(TARGETS cluster_bev_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

