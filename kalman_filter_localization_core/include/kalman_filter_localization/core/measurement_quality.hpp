// Copyright (c) 2026, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//  * Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef KALMAN_FILTER_LOCALIZATION__CORE__MEASUREMENT_QUALITY_HPP_
#define KALMAN_FILTER_LOCALIZATION__CORE__MEASUREMENT_QUALITY_HPP_

#include <cmath>
#include <cstdint>

namespace kalman_filter_localization
{
namespace core
{

enum class MeasurementRejectReason : std::uint8_t
{
  kNone = 0,
  kInvalidInput,
  kReceiverQuality,
  kInnovationMagnitude,
  kNis,
  kReacquisitionConsistency,
  kNumericalFailure,
};

struct MeasurementQualityResult
{
  bool accepted{false};
  MeasurementRejectReason reason{MeasurementRejectReason::kInvalidInput};
};

inline MeasurementQualityResult evaluateMeasurementQuality(
  const bool receiver_quality_valid, const double innovation_magnitude,
  const double nis, const double maximum_innovation_magnitude,
  const double maximum_nis)
{
  if (!std::isfinite(innovation_magnitude) ||
    (!std::isfinite(nis) && maximum_nis > 0.0) ||
    !std::isfinite(maximum_innovation_magnitude) || !std::isfinite(maximum_nis) ||
    maximum_innovation_magnitude < 0.0 || maximum_nis < 0.0)
  {
    return MeasurementQualityResult{};
  }
  if (!receiver_quality_valid) {
    return MeasurementQualityResult{false, MeasurementRejectReason::kReceiverQuality};
  }
  if (maximum_innovation_magnitude > 0.0 &&
    innovation_magnitude > maximum_innovation_magnitude)
  {
    return MeasurementQualityResult{false, MeasurementRejectReason::kInnovationMagnitude};
  }
  if (maximum_nis > 0.0 && nis > maximum_nis) {
    return MeasurementQualityResult{false, MeasurementRejectReason::kNis};
  }
  return MeasurementQualityResult{true, MeasurementRejectReason::kNone};
}

}  // namespace core
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__CORE__MEASUREMENT_QUALITY_HPP_
