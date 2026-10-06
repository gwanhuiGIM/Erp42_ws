#ifndef KKL_ALG_UNSCENTED_KALMAN_FILTER_HPP
#define KKL_ALG_UNSCENTED_KALMAN_FILTER_HPP

#include <functional>
#include <Eigen/Dense>
#include <hdl_localization/pose_system.hpp>

namespace kkl {
namespace alg {

template<typename T, int N, int M>
class UnscentedKalmanFilter {
public:
  typedef Eigen::Matrix<T, N, 1> VectorN;
  typedef Eigen::Matrix<T, M, 1> VectorM;
  typedef Eigen::Matrix<T, N, N> MatrixNN;
  typedef Eigen::Matrix<T, M, M> MatrixMM;
  typedef Eigen::Matrix<T, N, M> MatrixNM;
  typedef Eigen::Matrix<T, M, N> MatrixMN;

  UnscentedKalmanFilter()
  : mean_(VectorN::Zero()),
    cov_(MatrixNN::Identity()),
    process_noise(MatrixNN::Identity()),
    measurement_noise(MatrixMM::Identity()),
    lambda(1.0),
    alpha(0.5),
    beta(2.0)
  {
    wm.resize(2 * N + 1);
    wc.resize(2 * N + 1);
    sigma_points.resize(N, 2 * N + 1);

    lambda = alpha * alpha * (N + lambda) - N;

    wm[0] = lambda / (N + lambda);
    wc[0] = lambda / (N + lambda) + (1 - alpha * alpha + beta);
    for(int i=1; i<2*N+1; i++) {
      wm[i] = 1 / (2 * (N + lambda));
      wc[i] = 1 / (2 * (N + lambda));
    }
  }

  ~UnscentedKalmanFilter() {}

  void set_system(const hdl_localization::PoseSystem& new_system) {
    this->system = new_system;
  }
  
  void set_dt(double dt) {
    this->system.dt = dt;
  }

  void set_process_noise(const MatrixNN& new_process_noise) {
    this->process_noise = new_process_noise;
  }

  void set_measurement_noise(const MatrixMM& new_measurement_noise) {
    this->measurement_noise = new_measurement_noise;
  }

  void set_mean(const VectorN& new_mean) {
    this->mean_ = new_mean;
  }

  void set_cov(const MatrixNN& new_cov) {
    this->cov_ = new_cov;
  }

  const VectorN& mean() const { return mean_; }
  const MatrixNN& cov() const { return cov_; }

  void predict() {
    generate_sigma_points();

    for(int i=0; i<2*N+1; i++) {
      sigma_points.col(i) = system.f(sigma_points.col(i));
    }

    const auto& sigmas = sigma_points;
    VectorN mean_pred = VectorN::Zero();
    for(int i=0; i<2*N+1; i++) {
      mean_pred += wm[i] * sigmas.col(i);
    }

    MatrixNN cov_pred = MatrixNN::Zero();
    for(int i=0; i<2*N+1; i++) {
      VectorN diff = sigmas.col(i) - mean_pred;
      cov_pred += wc[i] * diff * diff.transpose();
    }
    cov_pred += process_noise;

    mean_ = mean_pred;
    cov_ = cov_pred;
  }

  void correct(const VectorM& measurement) {
    Eigen::Matrix<T, M, 2 * N + 1> Z;
    for(int i=0; i<2*N+1; i++) {
      Z.col(i) = system.h(sigma_points.col(i));
    }

    VectorM Z_mean = VectorM::Zero();
    for(int i=0; i<2*N+1; i++) {
      Z_mean += wm[i] * Z.col(i);
    }

    MatrixMM S = MatrixMM::Zero();
    for(int i=0; i<2*N+1; i++) {
      VectorM diff = Z.col(i) - Z_mean;
      S += wc[i] * diff * diff.transpose();
    }
    S += measurement_noise;

    MatrixNM C = MatrixNM::Zero();
    for(int i=0; i<2*N+1; i++) {
      C += wc[i] * (sigma_points.col(i) - mean_) * (Z.col(i) - Z_mean).transpose();
    }

    MatrixNM K = C * S.inverse();
    mean_ += K * (measurement - Z_mean);
    cov_ -= K * S * K.transpose();
  }

private:
  void generate_sigma_points() {
    Eigen::LLT<MatrixNN> llt( (N + lambda) * cov_ );
    MatrixNN L = llt.matrixL();

    sigma_points.col(0) = mean_;
    for(int i=0; i<N; i++) {
      sigma_points.col(i + 1)   = mean_ + L.col(i);
      sigma_points.col(i + N + 1) = mean_ - L.col(i);
    }
  }

private:
  hdl_localization::PoseSystem system;

  VectorN mean_;
  MatrixNN cov_;

  MatrixNN process_noise;
  MatrixMM measurement_noise;

  double lambda;
  double alpha;
  double beta;

  Eigen::Matrix<T, 1, 2 * N + 1> wm;
  Eigen::Matrix<T, 1, 2 * N + 1> wc;
  Eigen::Matrix<T, N, 2 * N + 1> sigma_points;
};

}
}

#endif

