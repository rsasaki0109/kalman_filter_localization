// Copyright (c) 2020, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef KALMAN_FILTER_LOCALIZATION__EKF_HPP_
#define KALMAN_FILTER_LOCALIZATION__EKF_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

class EKFEstimator
{
public:
  EKFEstimator()
  : P_(EigenMatrix9d::Identity() * 100),
    var_imu_w_{0.33},
    var_imu_acc_{0.33},
    tau_gyro_bias_{1.0}
  {
    /* x  = [p v q] = [x y z vx vy vz qx qy qz qw] */
    x_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
  }

/* state
* x  = [p v q] = [x y z vx vy vz qx qy qz qw]
* dx = [dp dv dth] = [dx dy dz dvx dvy dvz dthx dthy dthz]
*
* pos_k = pos_{k-1} + vel_k * dt + (1/2) * (Rot(q_{k-1}) acc_{k-1}^{imu} - g) *dt^2
* vel_k = vel_{k-1} + (Rot(quat_{k-1})) acc_{k-1}^{imu} - g) *dt
* quat_k = Rot(w_{k-1}^{imu}*dt)*quat_{k-1}
*
* covariance
* P_{k} = F_k P_{k-1} F_k^T + L Q_k L^T
*/
  void predictionUpdate(
    const double current_time_imu,
    const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & linear_acceleration
  )
  {
    double dt_imu = current_time_imu - previous_time_imu_;
    previous_time_imu_ = current_time_imu;
    if (dt_imu > 0.5 /* [sec] */) {
      std::cout << "imu time interval is too large" << std::endl;
      return;
    }

    Eigen::Quaterniond quat_wdt = Eigen::Quaterniond(
      Eigen::AngleAxisd(gyro.x() * dt_imu, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(gyro.y() * dt_imu, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(gyro.z() * dt_imu, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d acc = Eigen::Vector3d(
      linear_acceleration.x(),
      linear_acceleration.y(),
      linear_acceleration.z());

    // state
    Eigen::Quaterniond previous_quat =
      Eigen::Quaterniond(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
    Eigen::MatrixXd rot_mat = previous_quat.toRotationMatrix();

    // pos
    x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dt_imu * x_.segment(STATE::VX, 3) +
      0.5 * dt_imu * dt_imu * (rot_mat * acc - gravity_);
    // vel
    x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dt_imu * (rot_mat * acc - gravity_);
    // quat
    Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;
    x_.segment(STATE::QX, 4) = Eigen::Vector4d(
      predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());

    // F
    Eigen::MatrixXd F = EigenMatrix9d::Identity();
    F.block<3, 3>(0, 3) = dt_imu * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d acc_skew;
    acc_skew <<
      0, -acc(2), acc(1),
      acc(2), 0, -acc(0),
      -acc(1), acc(0), 0;
    F.block<3, 3>(3, 6) = rot_mat * (-acc_skew) * dt_imu;

    // Q
    Eigen::MatrixXd Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block<3, 3>(0, 0) = var_imu_acc_ * Q.block<3, 3>(0, 0);
    Q.block<3, 3>(3, 3) = var_imu_w_ * Q.block<3, 3>(3, 3);
    Q = Q * (dt_imu * dt_imu);

    // L
    Eigen::MatrixXd L = Eigen::Matrix<double, num_error_state_, 6>::Zero();
    L.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
    L.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

    P_ = F * P_ * F.transpose() + L * Q * L.transpose();
  }

/*
* y = pobs = [xobs yobs zobs]
*
* K = P_k H^T (H P_k H^T + R)^{-1}
*
* dx = K (y_k - p_k )
*
* p_x = p_{k-1} + dp_k
* v_k = v_{k-1} + dv_k
* q_k = Rot(dth) q_{k-1}
*
* P_k = (I - KH)*P_{k-1}
*/
  void observationUpdate(
    const Eigen::Vector3d & y,
    const Eigen::Vector3d & variance
  )
  {
    // error state
    Eigen::Matrix3d R;
    R <<
      variance.x(), 0, 0,
      0, variance.y(), 0,
      0, 0, variance.z();
    Eigen::MatrixXd H = Eigen::Matrix<double, 3, num_error_state_>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
    Eigen::VectorXd dx = K * (y - x_.segment(STATE::X, 3));

    // state
    x_.segment(STATE::X, 3) = x_.segment(STATE::X, 3) + dx.segment(ERROR_STATE::DX, 3);
    x_.segment(STATE::VX, 3) = x_.segment(STATE::VX, 3) + dx.segment(ERROR_STATE::DVX, 3);
    double norm_quat = sqrt(
      pow(dx(ERROR_STATE::DTHX), 2) +
      pow(dx(ERROR_STATE::DTHY), 2) +
      pow(dx(ERROR_STATE::DTHZ), 2));

    if (norm_quat < 1e-10) {
      Eigen::Quaterniond dq = Eigen::Quaterniond(cos(norm_quat / 2), 0, 0, 0);
      Eigen::Quaterniond q = Eigen::Quaterniond(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
      Eigen::Quaterniond q_new = q * dq;
      x_.segment(STATE::QX, 4) = Eigen::Vector4d(q_new.x(), q_new.y(), q_new.z(), q_new.w());
    } else {
      Eigen::Quaterniond dq = Eigen::Quaterniond(
        cos(norm_quat / 2),
        sin(norm_quat / 2) * dx(ERROR_STATE::DTHX) / norm_quat,
        sin(norm_quat / 2) * dx(ERROR_STATE::DTHY) / norm_quat,
        sin(norm_quat / 2) * dx(ERROR_STATE::DTHZ) / norm_quat);
      Eigen::Quaterniond q = Eigen::Quaterniond(x_(STATE::QW), x_(STATE::QX), x_(STATE::QY), x_(STATE::QZ));
      Eigen::Quaterniond q_new = q * dq;
      x_.segment(STATE::QX, 4) = Eigen::Vector4d(q_new.x(), q_new.y(), q_new.z(), q_new.w());
    }

    P_ = (EigenMatrix9d::Identity() - K * H) * P_;
  }

  void setTauGyroBias(const double tau_gyro_bias)
  {
    tau_gyro_bias_ = tau_gyro_bias;
  }

  void setVarImuGyro(const double var_imu_w)
  {
    var_imu_w_ = var_imu_w;
  }

  void setVarImuAcc(const double var_imu_acc)
  {
    var_imu_acc_ = var_imu_acc;
  }

  void setInitialX(Eigen::VectorXd x)
  {
    x_ = x;
  }

  Eigen::VectorXd getX()
  {
    return x_;
  }

  Eigen::MatrixXd getCoveriance()
  {
    return P_;
  }

  int getNumState()
  {
    return num_state_;
  }

private:
  double previous_time_imu_;
  double var_imu_w_;
  double var_imu_acc_;

  static const int num_state_{10};
  static const int num_error_state_{9};

  typedef Eigen::Matrix<double, num_error_state_, num_error_state_> EigenMatrix9d;

  Eigen::Matrix<double, num_state_, 1> x_;
  EigenMatrix9d P_;

  const Eigen::Vector3d gravity_{0, 0, 9.80665};

  double tau_gyro_bias_;

  enum STATE
  {
    X  = 0, Y = 1, Z = 2,
    VX = 3, VY = 4, VZ = 5,
    QX = 6, QY = 7, QZ = 8, QW = 9,
  };
  enum ERROR_STATE
  {
    DX   = 0, DY = 1, DZ = 2,
    DVX  = 3, DVY = 4, DVZ = 5,
    DTHX = 6, DTHY = 7, DTHZ = 8,
  };
};

#endif  // KALMAN_FILTER_LOCALIZATION__EKF_HPP_
