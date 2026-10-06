#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/pose_system.hpp>
#include <kkl/math/quaternion.hpp>

namespace hdl_localization {

PoseEstimator::PoseEstimator(const Eigen::Matrix<double, 6, 6>& process_noise, const Eigen::Matrix<double, 6, 6>& measurement_noise, const Eigen::Matrix<double, 6, 1>& initial_state, double cool_time_duration)
    : cool_time_duration(cool_time_duration) {
  
  PoseSystem system;
  ukf.set_system(system);
  ukf.set_process_noise(process_noise);
  ukf.set_measurement_noise(measurement_noise);
  ukf.set_mean(initial_state);
  
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  ukf.set_cov(cov);

  last_correction_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

PoseEstimator::~PoseEstimator() {}

void PoseEstimator::reset(const Eigen::Isometry3d& pose) {
  Eigen::Matrix<double, 6, 1> state = Eigen::Matrix<double, 6, 1>::Zero();
  state.head<3>() = pose.translation();
  state.tail<3>() = kkl::math::quaternion_to_euler(Eigen::Quaterniond(pose.linear()));

  ukf.set_mean(state);
  ukf.set_cov(Eigen::Matrix<double, 6, 6>::Identity() * 0.1);

  last_correction_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

Eigen::Isometry3d PoseEstimator::predict(const rclcpp::Time& stamp) {
  if (last_correction_stamp.seconds() == 0) {
    last_correction_stamp = stamp;
  }

  double dt = (stamp - last_correction_stamp).seconds();
  if (dt < 0.0) {
    last_correction_stamp = stamp;
    dt = 0.0;
  }
  
  ukf.set_dt(dt);
  ukf.predict();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = ukf.mean().head<3>();
  // BUGFIX: Convert quaternion to rotation matrix before assignment
  pose.linear() = kkl::math::euler_to_quaternion(Eigen::Vector3d(ukf.mean().tail<3>())).toRotationMatrix();

  return pose;
}

Eigen::Isometry3d PoseEstimator::correct(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose) {
  if ((stamp - last_correction_stamp).seconds() < cool_time_duration) {
    return predict(stamp);
  }
  
  last_correction_stamp = stamp;

  Eigen::Matrix<double, 6, 1> measurement = Eigen::Matrix<double, 6, 1>::Zero();
  measurement.head<3>() = pose.translation();
  measurement.tail<3>() = kkl::math::quaternion_to_euler(Eigen::Quaterniond(pose.linear()));

  ukf.correct(measurement);

  Eigen::Isometry3d corrected_pose = Eigen::Isometry3d::Identity();
  corrected_pose.translation() = ukf.mean().head<3>();
  // BUGFIX: Convert quaternion to rotation matrix before assignment
  corrected_pose.linear() = kkl::math::euler_to_quaternion(Eigen::Vector3d(ukf.mean().tail<3>())).toRotationMatrix();

  return corrected_pose;
}

}

