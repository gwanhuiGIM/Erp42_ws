#ifndef HDL_LOCALIZATION_HPP
#define HDL_LOCALIZATION_HPP

#include <memory>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

// PCL Headers
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/registration.h>

// Local Headers
#include "hdl_localization/pose_estimator.hpp"
#include "hdl_localization/msg/scan_matching_status.hpp"

namespace hdl_localization {

/**
 * @brief HdlLocalization is a class that estimates the sensor pose by aligning input point clouds to a globalmap.
 * @param PointT
 */
template<typename PointT>
class HdlLocalization : public rclcpp::Node {
public:
  /**
   * @brief constructor for ROS 2
   * @param options Node options
   */
  HdlLocalization(const rclcpp::NodeOptions& options);
  ~HdlLocalization();

private:
  /**
   * @brief callback for point clouds (ROS 2 style)
   * @param points_msg
   */
  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief callback for globalmap (ROS 2 style)
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief callback for initial pose (ROS 2 style)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief initialize parameters
   */
  void initialize_params();

  /**
   * @brief create a registration method
   * @return registration method
   */
  std::shared_ptr<pcl::Registration<PointT, PointT>> create_registration();

private:
  // ROS 2 Publishers and Subscribers
  typename rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  typename rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  typename rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  typename rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  typename rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  typename rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Parameters
  std::string points_topic;
  std::string odom_child_frame_id;
  bool specify_init_pose;
  std::string globalmap_path;
  bool use_globalmap_path;

  // Point clouds
  typename pcl::PointCloud<PointT>::Ptr globalmap;
  std::shared_ptr<pcl::Filter<PointT>> downsample_filter;

  // Registration method
  std::shared_ptr<pcl::Registration<PointT, PointT>> registration;

  // Pose estimator
  std::unique_ptr<PoseEstimator> pose_estimator;
};

}  // namespace hdl_localization

#endif // HDL_LOCALIZATION_HPP

