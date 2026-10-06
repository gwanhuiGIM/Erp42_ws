#include "hdl_localization/hdl_localization.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// GICP, NDT, etc. headers
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#ifdef USE_FAST_GICP
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>
#ifdef USE_FAST_GICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_gicp_cuda.hpp>
#endif
#endif

namespace hdl_localization {

template<typename PointT>
HdlLocalization<PointT>::HdlLocalization(const rclcpp::NodeOptions& options)
: rclcpp::Node("hdl_localization", options)
{
  initialize_params();

  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    points_topic, rclcpp::SensorDataQoS(), std::bind(&HdlLocalization::points_callback, this, std::placeholders::_1));
  globalmap_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/globalmap", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), std::bind(&HdlLocalization::globalmap_callback, this, std::placeholders::_1));
  initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::SystemDefaultsQoS(), std::bind(&HdlLocalization::initialpose_callback, this, std::placeholders::_1));

  pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS());
  aligned_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", rclcpp::SystemDefaultsQoS());
  status_pub = this->create_publisher<hdl_localization::msg::ScanMatchingStatus>("/status", rclcpp::SystemDefaultsQoS());
  
  tf_buffer.reset(new tf2_ros::Buffer(this->get_clock()));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));
  tf_broadcaster.reset(new tf2_ros::TransformBroadcaster(this));
}

template<typename PointT>
HdlLocalization<PointT>::~HdlLocalization() {}


template<typename PointT>
void HdlLocalization<PointT>::initialize_params() {
  points_topic = this->declare_parameter<std::string>("points_topic", "/velodyne_points");
  odom_child_frame_id = this->declare_parameter<std::string>("odom_child_frame_id", "base_link");

  double cool_time_duration = this->declare_parameter<double>("cool_time_duration", 0.5);
  specify_init_pose = this->declare_parameter<bool>("specify_init_pose", false);

  Eigen::Matrix<double, 6, 6> process_noise = Eigen::Matrix<double, 6, 6>::Identity();
  process_noise(0, 0) = this->declare_parameter<double>("acc_cov_x", 1.0);
  process_noise(1, 1) = this->declare_parameter<double>("acc_cov_y", 1.0);
  process_noise(2, 2) = this->declare_parameter<double>("acc_cov_z", 1.0);
  process_noise(3, 3) = this->declare_parameter<double>("acc_cov_roll", 1.0);
  process_noise(4, 4) = this->declare_parameter<double>("acc_cov_pitch", 1.0);
  process_noise(5, 5) = this->declare_parameter<double>("acc_cov_yaw", 1.0);
  
  Eigen::Matrix<double, 6, 6> measurement_noise = Eigen::Matrix<double, 6, 6>::Identity();
  measurement_noise.block<3, 3>(0, 0) *= this->declare_parameter<double>("measurement_noise_pos", 0.1);
  measurement_noise.block<3, 3>(3, 3) *= this->declare_parameter<double>("measurement_noise_rot", 0.1);


  Eigen::Matrix<double, 6, 1> init_pose = Eigen::Matrix<double, 6, 1>::Zero();
  if(specify_init_pose) {
    init_pose[0] = this->declare_parameter<double>("init_x", 0.0);
    init_pose[1] = this->declare_parameter<double>("init_y", 0.0);
    init_pose[2] = this->declare_parameter<double>("init_z", 0.0);
    init_pose[3] = this->declare_parameter<double>("init_roll", 0.0);
    init_pose[4] = this->declare_parameter<double>("init_pitch", 0.0);
    init_pose[5] = this->declare_parameter<double>("init_yaw", 0.0);
  }

  pose_estimator.reset(new PoseEstimator(process_noise, measurement_noise, init_pose, cool_time_duration));

  globalmap_path = this->declare_parameter<std::string>("globalmap_path", "");
  use_globalmap_path = this->declare_parameter<bool>("use_globalmap_path", false);
  if(use_globalmap_path && !globalmap_path.empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "load globalmap from " << globalmap_path);
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_path, *globalmap);
  }

  registration = create_registration();
  if(globalmap) {
    registration->setInputTarget(globalmap);
  }

  double downsample_resolution = this->declare_parameter<double>("downsample_resolution", 0.1);
  auto voxelgrid = std::make_shared<pcl::VoxelGrid<PointT>>();
  voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
  downsample_filter = voxelgrid;
}

template<typename PointT>
void HdlLocalization<PointT>::points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!globalmap) {
    RCLCPP_WARN(this->get_logger(), "globalmap has not been received");
    return;
  }

  const auto& stamp = msg->header.stamp;
  Eigen::Isometry3d predicted_pose = pose_estimator->predict(stamp);

  geometry_msgs::msg::TransformStamped sensor_to_base_trans;
  try {
    sensor_to_base_trans = tf_buffer->lookupTransform(odom_child_frame_id, msg->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException& e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "failed to look up transform from " << msg->header.frame_id << " to " << odom_child_frame_id << ": " << e.what());
    return;
  }
  Eigen::Isometry3d sensor_to_base = tf2::transformToEigen(sensor_to_base_trans);

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *cloud);

  typename pcl::PointCloud<PointT>::Ptr downsampled(new pcl::PointCloud<PointT>());
  downsample_filter->setInputCloud(cloud);
  downsample_filter->filter(*downsampled);

  if(downsampled->empty()) {
    RCLCPP_WARN(this->get_logger(), "downsampled cloud is empty");
    return;
  }
  
  typename pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration->setInputSource(downsampled);
  registration->align(*aligned, (predicted_pose * sensor_to_base).matrix().template cast<float>());

  auto status_msg = std::make_unique<hdl_localization::msg::ScanMatchingStatus>();
  status_msg->header = msg->header;
  status_msg->header.frame_id = "map";
  status_msg->has_converged = registration->hasConverged();
  status_msg->matching_error = registration->getFitnessScore();
  status_msg->inlier_fraction = 0.0;
  status_msg->relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().template cast<double>())).transform;
  status_pub->publish(std::move(status_msg));

  if(!registration->hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "scan matching has not converged");
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = odom_child_frame_id;
    odom.pose.pose = tf2::toMsg(predicted_pose);
    odom.pose.covariance.fill(1000.0);
    pose_pub->publish(odom);
    return;
  }

  Eigen::Isometry3d registration_pose(registration->getFinalTransformation().template cast<double>());
  Eigen::Isometry3d new_pose = registration_pose * sensor_to_base.inverse();
  
  Eigen::Isometry3d corrected_pose = pose_estimator->correct(stamp, new_pose);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = odom_child_frame_id;
  odom.pose.pose = tf2::toMsg(corrected_pose);
  const auto& cov = pose_estimator->cov();
  for(int i=0; i<6; i++) {
    for(int j=0; j<6; j++) {
      odom.pose.covariance[i * 6 + j] = cov(i, j);
    }
  }
  pose_pub->publish(odom);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = odom_child_frame_id;
  transform_stamped.transform = tf2::eigenToTransform(corrected_pose).transform;
  tf_broadcaster->sendTransform(transform_stamped);

  sensor_msgs::msg::PointCloud2 aligned_msg;
  pcl::toROSMsg(*aligned, aligned_msg);
  aligned_msg.header.stamp = msg->header.stamp;
  aligned_msg.header.frame_id = "map";
  aligned_pub->publish(aligned_msg);
}

template<typename PointT>
void HdlLocalization<PointT>::globalmap_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Global map received!");
  globalmap.reset(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *globalmap);
  registration->setInputTarget(globalmap);
}

template<typename PointT>
void HdlLocalization<PointT>::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Initial pose received!");
  Eigen::Isometry3d pose;
  tf2::fromMsg(msg->pose.pose, pose);
  pose_estimator->reset(pose);
}

template<typename PointT>
std::shared_ptr<pcl::Registration<PointT, PointT>> HdlLocalization<PointT>::create_registration() {
  std::string reg_method = this->declare_parameter<std::string>("reg_method", "NDT");
  std::string ndt_neighbor_search_method = this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");

  if(reg_method == "GICP") {
    RCLCPP_INFO(this->get_logger(), "registration: GICP");
    auto gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>();
    gicp->setTransformationEpsilon(this->declare_parameter<double>("transformation_epsilon", 0.01));
    gicp->setMaximumIterations(this->declare_parameter<int>("maximum_iterations", 64));
    gicp->setUseReciprocalCorrespondences(this->declare_parameter<bool>("use_reciprocal_correspondences", false));
    gicp->setCorrespondenceRandomness(this->declare_parameter<int>("correspondence_randomness", 20));
    gicp->setMaximumOptimizerIterations(this->declare_parameter<int>("maximum_optimizer_iterations", 20));
    return gicp;
  }
#ifdef USE_FAST_GICP
  else if(reg_method == "F_GICP") {
    RCLCPP_INFO(this->get_logger(), "registration: F_GICP");
    auto gicp = std::make_shared<fast_gicp::FastGICP<PointT, PointT>>();
    gicp->setNumThreads(this->declare_parameter<int>("gicp_num_threads", 0));
    gicp->setTransformationEpsilon(this->declare_parameter<double>("transformation_epsilon", 0.01));
    gicp->setMaximumIterations(this->declare_parameter<int>("maximum_iterations", 64));
    gicp->setCorrespondenceRandomness(this->declare_parameter<int>("correspondence_randomness", 20));
    return gicp;
  } else if(reg_method == "F_VGICP") {
    RCLCPP_INFO(this->get_logger(), "registration: F_VGICP");
    auto vgicp = std::make_shared<fast_gicp::FastVGICP<PointT, PointT>>();
    vgicp->setNumThreads(this->declare_parameter<int>("gicp_num_threads", 0));
    vgicp->setResolution(this->declare_parameter<double>("vgicp_resolution", 1.0));
    vgicp->setTransformationEpsilon(this->declare_parameter<double>("transformation_epsilon", 0.01));
    vgicp->setMaximumIterations(this->declare_parameter<int>("maximum_iterations", 64));
    vgicp->setCorrespondenceRandomness(this->declare_parameter<int>("correspondence_randomness", 20));
    return vgicp;
  }
#ifdef USE_FAST_GICP_CUDA
  else if (reg_method == "NDT_CUDA") {
    RCLCPP_INFO(this->get_logger(), "registration: NDT_CUDA");
    auto ndt = std::make_shared<fast_gicp::NDTCuda<PointT, PointT>>();
    ndt->setResolution(this->declare_parameter<double>("resolution", 1.0));
    if (ndt_neighbor_search_method == "DIRECT1") {
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    } else if (ndt_neighbor_search_method == "DIRECT7") {
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    } else {
      ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::KDTREE);
    }
    ndt->setTransformationEpsilon(this->declare_parameter<double>("transformation_epsilon", 0.01));
    ndt->setMaximumIterations(this->declare_parameter<int>("maximum_iterations", 64));
    return ndt;
  }
#endif
#endif
  else {
    if(reg_method != "NDT") {
      RCLCPP_WARN(this->get_logger(), "invalid registration method:%s", reg_method.c_str());
      RCLCPP_WARN(this->get_logger(), "NDT will be used");
    }

    RCLCPP_INFO(this->get_logger(), "registration: NDT");
    auto ndt = std::make_shared<pcl::NormalDistributionsTransform<PointT, PointT>>();
    ndt->setResolution(this->declare_parameter<double>("resolution", 1.0));
    ndt->setStepSize(this->declare_parameter<double>("step_size", 0.1));
    ndt->setTransformationEpsilon(this->declare_parameter<double>("transformation_epsilon", 0.01));
    ndt->setMaximumIterations(this->declare_parameter<int>("maximum_iterations", 64));
    return ndt;
  }
}

// Explicit template instantiation
template class HdlLocalization<pcl::PointXYZI>;

}  // namespace hdl_localization

