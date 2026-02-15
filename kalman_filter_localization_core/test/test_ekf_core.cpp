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

#include <gtest/gtest.h>

#include <limits>

#include <kalman_filter_localization/core/ekf.hpp>

TEST(EKFEstimatorCore, PredictionUpdateDtStatus)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateDt(0.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);
  EXPECT_EQ(
    ekf.predictionUpdateDt(-0.1, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);
  EXPECT_EQ(
    ekf.predictionUpdateDt(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kDtTooLarge);
  EXPECT_EQ(
    ekf.predictionUpdateDt(0.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}

TEST(EKFEstimatorCore, ObservationUpdateValidation)
{
  EKFEstimator ekf;

  const Eigen::Vector3d y_valid(1.0, 2.0, 3.0);
  const Eigen::Vector3d variance_valid(1.0, 1.0, 1.0);

  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_valid, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kUpdated);

  const Eigen::Vector3d variance_bad(0.0, 1.0, 1.0);
  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_valid, variance_bad),
    EKFEstimator::ObservationUpdateStatus::kInvalidVariance);

  Eigen::Vector3d y_bad = y_valid;
  y_bad.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    ekf.observationUpdateWithStatus(y_bad, variance_valid),
    EKFEstimator::ObservationUpdateStatus::kInvalidMeasurement);
}

TEST(EKFEstimatorCore, PredictionUpdateWithStatusTimeBaseAndReset)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kNonPositiveDt);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(10.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);

  ekf.resetImuTimeBase();
  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(20.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);
}

TEST(EKFEstimatorCore, PredictionUpdateWithStatusRecoversAfterLargeDt)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(0.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kSkippedNoTimeBase);
  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kDtTooLarge);

  EXPECT_EQ(
    ekf.predictionUpdateWithStatus(1.01, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}

TEST(EKFEstimatorCore, SetInitialXCheckedValidatesSize)
{
  EKFEstimator ekf;

  Eigen::VectorXd x_bad(ekf.getNumState() - 1);
  x_bad.setZero();
  EXPECT_FALSE(ekf.setInitialXChecked(x_bad));

  Eigen::VectorXd x_ok(ekf.getNumState());
  x_ok.setZero();
  // Last element is qw.
  x_ok(ekf.getNumState() - 1) = 1.0;
  EXPECT_TRUE(ekf.setInitialXChecked(x_ok));
}

TEST(EKFEstimatorCore, MaxPredictionDtSecConfig)
{
  EKFEstimator ekf;
  const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  const Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_DOUBLE_EQ(ekf.getMaxPredictionDtSec(), 0.5);
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(0.0));
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(-1.0));
  EXPECT_FALSE(ekf.setMaxPredictionDtSec(std::numeric_limits<double>::infinity()));
  EXPECT_TRUE(ekf.setMaxPredictionDtSec(1.5));
  EXPECT_DOUBLE_EQ(ekf.getMaxPredictionDtSec(), 1.5);

  EXPECT_EQ(
    ekf.predictionUpdateDt(1.0, gyro, acc),
    EKFEstimator::PredictionUpdateStatus::kUpdated);
}
