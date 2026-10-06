#ifndef HDL_LOCALIZATION_POSE_ESTIMATOR_HPP
#define HDL_LOCALIZATION_POSE_ESTIMATOR_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <kkl/alg/unscented_kalman_filter.hpp>
#include <Eigen/Dense>

namespace hdl_localization {

class PoseEstimator {
public:
  using Ptr = std::shared_ptr<PoseEstimator>;

  PoseEstimator(const Eigen::Matrix<double, 6, 6>& process_noise, const Eigen::Matrix<double, 6, 6>& measurement_noise, const Eigen::Matrix<double, 6, 1>& initial_state, double cool_time_duration = 1.0);
  ~PoseEstimator();

  void reset(const Eigen::Isometry3d& pose);

  Eigen::Isometry3d predict(const rclcpp::Time& stamp);

  // BUGFIX: correct 함수의 파라미터에 타임스탬프를 추가합니다.
  Eigen::Isometry3d correct(const rclcpp::Time& stamp, const Eigen::Isometry3d& pose);

  const Eigen::Matrix<double, 6, 1>& mean() const {
    return ukf.mean();
  }

  const Eigen::Matrix<double, 6, 6>& cov() const {
    return ukf.cov();
  }

private:
  rclcpp::Time last_correction_stamp;
  double cool_time_duration;

  // BUGFIX: UKF의 타입을 올바르게 명시합니다.
  kkl::alg::UnscentedKalmanFilter<double, 6, 6> ukf;
};

}

#endif

