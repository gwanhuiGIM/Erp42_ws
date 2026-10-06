#ifndef KKL_MATH_QUATERNION_HPP
#define KKL_MATH_QUATERNION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kkl {
namespace math {

/**
 * @brief convert a quaternion to euler angles
 * @param quat  quaternion
 * @return euler angles (roll, pitch, yaw)
 */
template<typename T>
Eigen::Matrix<T, 3, 1> quaternion_to_euler(const Eigen::Quaternion<T>& quat) {
  Eigen::Matrix<T, 3, 3> mat = quat.toRotationMatrix();
  return mat.eulerAngles(0, 1, 2);
}

/**
 * @brief convert euler angles to a quaternion
 * @param euler  euler angles (roll, pitch, yaw)
 * @return quaternion
 */
template<typename T>
Eigen::Quaternion<T> euler_to_quaternion(const Eigen::Matrix<T, 3, 1>& euler) {
  return Eigen::AngleAxis<T>(euler[0], Eigen::Matrix<T, 3, 1>::UnitX())
         * Eigen::AngleAxis<T>(euler[1], Eigen::Matrix<T, 3, 1>::UnitY())
         * Eigen::AngleAxis<T>(euler[2], Eigen::Matrix<T, 3, 1>::UnitZ());
}

}
}

#endif

