#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp_components/register_node_macro.hpp"

namespace hdl_localization {

class GlobalmapServer : public rclcpp::Node {
public:
  GlobalmapServer(const rclcpp::NodeOptions& options) : Node("globalmap_server", options) {
    // ROS 2 파라미터 선언 API를 사용하여 pcd 파일 경로를 가져옵니다.
    std::string globalmap_path = this->declare_parameter<std::string>("globalmap_path", "");
    if (globalmap_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'globalmap_path' is not set. Please provide a path to the .pcd file.");
      return;
    }

    // 포인트 클라우드 파일을 로드합니다.
    pcl::PointCloud<pcl::PointXYZI>::Ptr globalmap(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(globalmap_path, *globalmap) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", globalmap_path.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %ld data points from %s", globalmap->size(), globalmap_path.c_str());

    // transient_local QoS를 사용하여 마지막 메시지를 유지(latch)하는 퍼블리셔를 생성합니다.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", qos);

    // PCL 포인트 클라우드를 ROS 2 메시지로 변환합니다.
    auto globalmap_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*globalmap, *globalmap_msg);
    globalmap_msg->header.frame_id = "map";
    globalmap_msg->header.stamp = this->get_clock()->now();

    // 맵을 발행합니다.
    globalmap_pub_->publish(std::move(globalmap_msg));
    RCLCPP_INFO(this->get_logger(), "Global map published.");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
};

} // namespace hdl_localization

// 컴포넌트로 등록하여 launch 파일에서 로드할 수 있도록 합니다.
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServer)

