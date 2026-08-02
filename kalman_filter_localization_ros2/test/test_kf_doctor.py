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

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'kf_doctor.py'
SPEC = importlib.util.spec_from_file_location('kf_doctor', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def imu_message(gyro=(0.0, 0.0, 0.0), acceleration=(0.0, 0.0, 9.80665)):
    """Build the subset of sensor_msgs/Imu used by the doctor."""
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=0), frame_id='imu_link'),
        angular_velocity=SimpleNamespace(x=gyro[0], y=gyro[1], z=gyro[2]),
        linear_acceleration=SimpleNamespace(
            x=acceleration[0], y=acceleration[1], z=acceleration[2]))


def test_imu_statistics_detects_stationary_window_and_rate():
    record = MODULE.TopicRecord('/imu', 'sensor_msgs/msg/Imu')
    for index in range(100):
        stamp = 10.0 + index * 0.01
        message = imu_message(gyro=(0.001 * (index % 2), 0.0, 0.0))
        record.samples.append((stamp, message))
        record.stamps.append(stamp)
        record.frames.add('imu_link')

    result = MODULE.imu_statistics(record)

    assert result['sample_count'] == 100
    assert result['stationary_sample_count'] == 100
    assert 99.0 < result['rate_hz'] < 101.0
    assert result['heuristic_continuous_psd']['var_imu_w'] is not None
    assert result['stationary_accel_norm_mps2'] == 9.80665


def test_analyze_emits_profile_skeleton_and_missing_sensor_warning(monkeypatch):
    record = MODULE.TopicRecord('/imu', 'sensor_msgs/msg/Imu')
    for index in range(60):
        stamp = 1.0 + index * 0.01
        record.samples.append((stamp, imu_message()))
        record.stamps.append(stamp)
    topic_types = {
        '/imu': 'sensor_msgs/msg/Imu',
        '/initial_pose': 'geometry_msgs/msg/PoseStamped',
    }
    selections = {
        'imu': {'requested': '', 'selected': '/imu', 'selection': 'auto'},
        'gnss': {'requested': '', 'selected': None, 'selection': 'not_found'},
        'wheel': {'requested': '', 'selected': None, 'selection': 'not_found'},
        'odom': {'requested': '', 'selected': None, 'selection': 'not_found'},
        'initial_pose': {
            'requested': '', 'selected': '/initial_pose', 'selection': 'auto'},
    }
    monkeypatch.setattr(
        MODULE, 'read_bag', lambda *args, **kwargs: (topic_types, selections, {'/imu': record}))

    report = MODULE.analyze(Path('/tmp/input_bag'), {
        'imu': '', 'gnss': '', 'wheel': '', 'odom': '', 'initial_pose': ''})

    assert report['summary']['status'] == 'warn'
    assert report['selection']['imu']['message_type'] == 'sensor_msgs/msg/Imu'
    assert report['profile_skeleton']['ekf_localization']['ros__parameters']['imu_topic'] == '/imu'
    assert any(item['code'] == 'missing_gnss' for item in report['checks'])


def test_profile_skeleton_maps_covariance_initial_pose_topic():
    selections = {
        'imu': {'selected': '/imu'},
        'gnss': {'selected': None},
        'wheel': {'selected': None},
        'odom': {'selected': '/odometry/gps'},
        'initial_pose': {
            'selected': '/initialpose',
            'message_type': 'geometry_msgs/msg/PoseWithCovarianceStamped',
        },
    }
    profile = MODULE.profile_skeleton(
        selections,
        {'stationary_sample_count': 0, 'heuristic_continuous_psd': {}},
        MODULE.GRAVITY_MPS2)
    parameters = profile['ekf_localization']['ros__parameters']

    assert parameters['initial_pose_covariance_topic'] == '/initialpose'
    assert 'initial_pose_topic' not in parameters
