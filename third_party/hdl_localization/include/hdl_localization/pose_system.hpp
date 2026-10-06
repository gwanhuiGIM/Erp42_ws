#ifndef HDL_LOCALIZATION_POSE_SYSTEM_HPP
#define HDL_LOCALIZATION_POSE_SYSTEM_HPP

#include <Eigen/Dense>

namespace hdl_localization {

class PoseSystem {
public:
  typedef Eigen::Matrix<double, 6, 1> VectorXt;

public:
  PoseSystem() {
    dt = 0.01;
  }

  // 시스템 모델 f(x) - 현재는 정적 모델(상태가 변하지 않음)을 가정합니다.
  VectorXt f(const VectorXt& state) const {
    return state;
  }

  // 측정 모델 h(x) - 상태 전체를 직접 측정할 수 있다고 가정합니다.
  VectorXt h(const VectorXt& state) const {
    return state;
  }

  double dt;
};

}

#endif

