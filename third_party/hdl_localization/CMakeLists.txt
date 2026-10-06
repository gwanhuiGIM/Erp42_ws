cmake_minimum_required(VERSION 3.8)
project(hdl_localization)

# C++17 표준을 사용하도록 설정합니다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 필요한 ROS 2 패키지를 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

# 외부 의존성 패키지를 찾습니다.
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(ndt_omp)
find_package(fast_gicp)
find_package(hdl_global_localization)

# package.xml에서 빌드 의존성을 자동으로 찾습니다.
ament_auto_find_build_dependencies()

# ScanMatchingStatus.msg 파일로부터 C++ 헤더를 생성합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanMatchingStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 생성된 메시지의 C++ 타입 서포트 타겟을 가져옵니다.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# PCL과 로컬 include 디렉토리를 포함합니다.
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# OpenMP를 찾았을 경우 관련 플래그를 추가합니다.
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# hdl_localization 컴포넌트 라이브러리를 정의합니다.
add_library(hdl_localization_component SHARED
  src/hdl_localization/pose_estimator.cpp
  apps/hdl_localization.cpp
)
target_include_directories(hdl_localization_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hdl_localization_component
  rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs
  nav_msgs std_srvs tf2_ros tf2_eigen pcl_conversions
  ndt_omp fast_gicp hdl_global_localization
)
# BUGFIX: 생성된 메시지 라이브러리에 명시적으로 링크하여 빌드 순서 문제를 해결합니다.
target_link_libraries(hdl_localization_component
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)

# 컴포넌트를 노드로 등록합니다.
rclcpp_components_register_node(
  hdl_localization_component
  PLUGIN "hdl_localization::HdlLocalization<pcl::PointXYZI>"
  EXECUTABLE hdl_localization_node
)

# globalmap_server 컴포넌트 라이브러리를 정의합니다.
add_library(globalmap_server_component SHARED
  apps/globalmap_server.cpp
)
target_include_directories(globalmap_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(globalmap_server_component
  rclcpp rclcpp_components sensor_msgs pcl_conversions
)
target_link_libraries(globalmap_server_component
  ${PCL_LIBRARIES}
)

# 컴포넌트를 노드로 등록합니다.
rclcpp_components_register_node(
  globalmap_server_component
  PLUGIN "hdl_localization::GlobalmapServer"
  EXECUTABLE globalmap_server
)

# 설치 규칙을 정의합니다.
install(
  DIRECTORY launch rviz param
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS hdl_localization_component globalmap_server_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/plot_status.py
  DESTINATION lib/${PROJECT_NAME}
)

# 패키징을 완료합니다.
ament_package()

