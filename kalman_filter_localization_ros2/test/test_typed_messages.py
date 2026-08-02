# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from kalman_filter_localization_msgs.msg import EstimatorStatus
from kalman_filter_localization_msgs.msg import MeasurementQuality
from kalman_filter_localization_msgs.msg import ObservabilityStatus
from kalman_filter_localization_msgs.msg import ReplayTiming


def test_typed_message_constants_and_defaults():
    status = EstimatorStatus()
    quality = MeasurementQuality()
    observability = ObservabilityStatus()
    replay = ReplayTiming()

    assert status.HEALTH_OK == 1
    assert status.MODE_REACQUISITION == 5
    assert status.health == EstimatorStatus.HEALTH_UNKNOWN
    assert not status.sensor_fault_isolation_enabled
    assert not status.gnss_isolated
    assert not quality.accepted
    assert observability.nhc_variance_scale == 0.0
    assert observability.vehicle_model == ''
    assert not observability.nhc_vertical_constrained
    assert not replay.applied


def test_typed_message_fields_are_assignable():
    quality = MeasurementQuality()
    quality.source = 'gnss_position'
    quality.source_id = 1
    quality.accepted = True
    quality.reject_reason_text = 'none'
    quality.raw_nis = 2.5
    quality.used_nis = 2.5
    quality.variance_scale = 1.0
    status = EstimatorStatus()
    status.sensor_fault_isolation_enabled = True
    status.gnss_isolated = True
    status.gnss_fault_events = 1

    assert quality.source == 'gnss_position'
    assert quality.source_id == 1
    assert quality.accepted
    assert quality.raw_nis == 2.5
    assert status.gnss_isolated
    assert status.gnss_fault_events == 1
