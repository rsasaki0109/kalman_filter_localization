#!/usr/bin/env python3
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
"""Inspect a rosbag2 and generate a localization configuration skeleton."""

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import statistics
import sys

import yaml


GRAVITY_MPS2 = 9.80665
DEFAULT_MAX_SAMPLES_PER_TOPIC = 100000
ROLE_CANDIDATES = {
    'imu': ('sensor_msgs/msg/Imu',),
    'gnss': ('geometry_msgs/msg/PoseStamped', 'sensor_msgs/msg/NavSatFix'),
    'wheel': ('geometry_msgs/msg/TwistWithCovarianceStamped',),
    'odom': ('nav_msgs/msg/Odometry',),
    'initial_pose': (
        'geometry_msgs/msg/PoseStamped',
        'geometry_msgs/msg/PoseWithCovarianceStamped',),
}


@dataclass
class TopicRecord:
    """Observed messages and compact statistics for one topic."""

    name: str
    message_type: str
    message_count: int = 0
    samples: list = field(default_factory=list)
    stamps: list = field(default_factory=list)
    recorded_stamps: list = field(default_factory=list)
    frames: set = field(default_factory=set)
    covariance_invalid: int = 0
    covariance_present: int = 0


def finite_or_none(value):
    """Return JSON-safe finite values and represent unavailable values as null."""
    return value if value is not None and math.isfinite(value) else None


def stamp_seconds(message, recorded_stamp_ns):
    """Read a message header stamp, falling back to the rosbag record stamp."""
    header = getattr(message, 'header', None)
    if header is not None:
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
            if math.isfinite(value) and value != 0.0:
                return value
    return float(recorded_stamp_ns) * 1.0e-9


def vector_values(vector):
    """Return a finite xyz tuple when a ROS vector is available."""
    if vector is None:
        return None
    values = (float(vector.x), float(vector.y), float(vector.z))
    return values if all(math.isfinite(value) for value in values) else None


def covariance_arrays(message):
    """Collect covariance arrays exposed by common ROS sensor messages."""
    arrays = []
    for name in (
            'orientation_covariance', 'angular_velocity_covariance',
            'linear_acceleration_covariance', 'position_covariance'):
        value = getattr(message, name, None)
        if value is not None:
            arrays.append(list(value))
    pose = getattr(message, 'pose', None)
    if pose is not None and hasattr(pose, 'covariance'):
        arrays.append(list(pose.covariance))
    twist = getattr(message, 'twist', None)
    if twist is not None and hasattr(twist, 'covariance'):
        arrays.append(list(twist.covariance))
    return arrays


def covariance_is_valid(array):
    """Validate a ROS covariance array, respecting -1 unknown covariance markers."""
    if not array:
        return True
    if float(array[0]) < 0.0:
        return True
    if not all(math.isfinite(float(value)) for value in array):
        return False
    size = int(round(math.sqrt(len(array))))
    if size * size != len(array):
        return True
    diagonal = [float(array[index * size + index]) for index in range(size)]
    return all(value > 0.0 for value in diagonal)


def topic_rate(stamps):
    """Return median rate and gap statistics from sensor timestamps."""
    positive_dts = [right - left for left, right in zip(stamps, stamps[1:])
                    if right - left > 0.0 and math.isfinite(right - left)]
    if not positive_dts:
        return {
            'median_dt_sec': None, 'rate_hz': None, 'max_gap_sec': None,
            'negative_or_duplicate_intervals': max(0, len(stamps) - 1),
        }
    negative_or_duplicate = sum(
        1 for left, right in zip(stamps, stamps[1:]) if right <= left)
    median_dt = statistics.median(positive_dts)
    return {
        'median_dt_sec': median_dt,
        'rate_hz': 1.0 / median_dt,
        'max_gap_sec': max(positive_dts),
        'negative_or_duplicate_intervals': negative_or_duplicate,
    }


def summarize_topic(record):
    """Convert a topic record into a JSON-safe summary."""
    timing = topic_rate(record.stamps)
    return {
        'type': record.message_type,
        'message_count': record.message_count,
        'sample_count': len(record.samples),
        'stamp_start': finite_or_none(min(record.stamps) if record.stamps else None),
        'stamp_end': finite_or_none(max(record.stamps) if record.stamps else None),
        'recorded_stamp_start': finite_or_none(
            min(record.recorded_stamps) if record.recorded_stamps else None),
        'recorded_stamp_end': finite_or_none(
            max(record.recorded_stamps) if record.recorded_stamps else None),
        'frames': sorted(record.frames),
        'covariance_present_samples': record.covariance_present,
        'covariance_invalid_samples': record.covariance_invalid,
        'timing': timing,
    }


def imu_statistics(record, gravity=GRAVITY_MPS2):
    """Estimate IMU rate and stationary white-noise values for a profile hint."""
    gyro = []
    acceleration = []
    for stamp, message in record.samples:
        gyro_value = vector_values(getattr(message, 'angular_velocity', None))
        acceleration_value = vector_values(getattr(message, 'linear_acceleration', None))
        if gyro_value is not None and acceleration_value is not None:
            gyro.append((stamp, gyro_value))
            acceleration.append((stamp, acceleration_value))
    timing = topic_rate([sample[0] for sample in gyro])
    rate = timing['rate_hz']
    stationary = []
    if rate is not None:
        for (_, gyro_value), (_, acceleration_value) in zip(gyro, acceleration):
            gyro_norm = math.sqrt(sum(value * value for value in gyro_value))
            acceleration_norm = math.sqrt(sum(value * value for value in acceleration_value))
            if gyro_norm <= 0.03 and abs(acceleration_norm - gravity) <= 0.5:
                stationary.append((gyro_value, acceleration_value))

    def component_std(values, component):
        if len(values) < 2:
            return None
        return statistics.pstdev(value[component] for value in values)

    gyro_std = [component_std([item[0] for item in stationary], index)
                for index in range(3)]
    accel_std = [component_std([item[1] for item in stationary], index)
                 for index in range(3)]
    gyro_density = [value * math.sqrt(rate) if value is not None and rate else None
                    for value in gyro_std]
    accel_density = [value * math.sqrt(rate) if value is not None and rate else None
                     for value in accel_std]
    acceleration_norms = [
        math.sqrt(sum(value * value for value in item[1])) for item in stationary]
    return {
        'sample_count': len(record.samples),
        'stationary_sample_count': len(stationary),
        'rate_hz': finite_or_none(rate),
        'gyro_std_radps': [finite_or_none(value) for value in gyro_std],
        'accel_std_mps2': [finite_or_none(value) for value in accel_std],
        'stationary_accel_norm_mps2': finite_or_none(
            statistics.median(acceleration_norms) if acceleration_norms else None),
        'heuristic_white_noise_density': {
            'gyro_radps_sqrt_hz': [finite_or_none(value) for value in gyro_density],
            'accel_mps2_sqrt_hz': [finite_or_none(value) for value in accel_density],
        },
        'heuristic_continuous_psd': {
            'var_imu_w': finite_or_none(
                statistics.mean(value * value for value in gyro_density)
                if any(value is not None for value in gyro_density) else None),
            'var_imu_acc': finite_or_none(
                statistics.mean(value * value for value in accel_density)
                if any(value is not None for value in accel_density) else None),
        },
    }


def candidate_topic(topic_types, requested, role):
    """Select a requested topic or an obvious type/name-compatible candidate."""
    if requested:
        if requested in topic_types:
            return requested, 'requested'
        return None, 'requested_missing'
    candidates = []
    for name, message_type in sorted(topic_types.items()):
        if message_type not in ROLE_CANDIDATES[role]:
            continue
        lower_name = name.lower()
        score = 0
        if role == 'initial_pose' and not any(
                token in lower_name for token in ('initial', 'set_pose')):
            continue
        if role == 'imu' and 'imu' in lower_name:
            score += 4
        if role == 'gnss' and any(token in lower_name for token in ('gnss', 'gps', 'fix')):
            score += 4
        if role == 'wheel' and any(token in lower_name for token in ('wheel', 'speed', 'twist')):
            score += 4
        if role == 'odom' and 'odom' in lower_name:
            score += 4
        if role == 'initial_pose' and any(
                token in lower_name for token in ('initial', 'set_pose')):
            score += 8
        candidates.append((score, name))
    if not candidates:
        return None, 'not_found'
    return max(candidates, key=lambda item: (item[0], item[1]))[1], 'auto'


def load_bag_metadata(path):
    """Read optional rosbag2 metadata, including recorded QoS profiles."""
    metadata_path = Path(path) / 'metadata.yaml'
    if not metadata_path.is_file():
        return {}
    with metadata_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    information = document.get('rosbag2_bagfile_information', {})
    topics = {}
    for item in information.get('topics_with_message_count', []):
        topic_metadata = item.get('topic_metadata', {})
        name = topic_metadata.get('name')
        if name:
            topics[name] = {
                'type': topic_metadata.get('type'),
                'serialization_format': topic_metadata.get('serialization_format'),
                'offered_qos_profiles': topic_metadata.get('offered_qos_profiles') or [],
                'message_count': item.get('message_count'),
            }
    return {
        'ros_distro': information.get('ros_distro'),
        'storage_identifier': information.get('storage_identifier'),
        'duration_nanoseconds': information.get('duration', {}).get('nanoseconds'),
        'starting_time_nanoseconds': information.get(
            'starting_time', {}).get('nanoseconds_since_epoch'),
        'message_count': information.get('message_count'),
        'topics': topics,
    }


def read_bag(path, topic_requests, max_samples=DEFAULT_MAX_SAMPLES_PER_TOPIC):
    """Read selected message samples and topic metadata from rosbag2."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('bag inspection requires a sourced ROS 2 environment') from error

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    selections = {}
    records = {}
    for role, requested in topic_requests.items():
        selected, selection = candidate_topic(topic_types, requested, role)
        selections[role] = {
            'requested': requested,
            'selected': selected,
            'selection': selection,
        }
        if selected is not None and selected not in records:
            records[selected] = TopicRecord(selected, topic_types[selected])
    message_types = {
        name: get_message(record.message_type) for name, record in records.items()}
    while reader.has_next():
        topic, data, recorded_stamp = reader.read_next()
        if topic not in records:
            continue
        record = records[topic]
        record.message_count += 1
        if len(record.samples) >= max_samples:
            continue
        message = deserialize_message(data, message_types[topic])
        stamp = stamp_seconds(message, recorded_stamp)
        record.stamps.append(stamp)
        record.recorded_stamps.append(float(recorded_stamp) * 1.0e-9)
        header = getattr(message, 'header', None)
        frame = getattr(header, 'frame_id', '') if header is not None else ''
        if frame:
            record.frames.add(frame)
        arrays = covariance_arrays(message)
        if arrays:
            record.covariance_present += 1
            if not all(covariance_is_valid(array) for array in arrays):
                record.covariance_invalid += 1
        record.samples.append((stamp, message))
    return topic_types, selections, records


def check(severity, code, message, details=None):
    """Build one stable diagnostic record."""
    result = {'severity': severity, 'code': code, 'message': message}
    if details:
        result['details'] = details
    return result


def profile_skeleton(selections, imu, gravity):
    """Generate a conservative parameter skeleton, not a final tuned profile."""
    parameters = {
        'reference_frame_id': 'map',
        'robot_frame_id': 'base_link',
        'propagation_model': 'exact',
        'enable_stationary_initialization': imu['stationary_sample_count'] >= 50,
        'stationary_initialization_min_samples': min(
            max(50, imu['stationary_sample_count']), 200),
        'gravity_mps2': gravity,
        'use_gnss': selections['gnss']['selected'] is not None,
        'use_wheel_speed': selections['wheel']['selected'] is not None,
        'use_odom': selections['odom']['selected'] is not None,
    }
    if selections['imu']['selected'] is not None:
        parameters['imu_topic'] = selections['imu']['selected']
    if selections['gnss']['selected'] is not None:
        gnss_type = selections['gnss'].get('message_type')
        if gnss_type == 'sensor_msgs/msg/NavSatFix':
            parameters['gnss_input_type'] = 'navsatfix'
            parameters['gnss_navsatfix_topic'] = selections['gnss']['selected']
        else:
            parameters['gnss_input_type'] = 'pose'
            parameters['gnss_pose_topic'] = selections['gnss']['selected']
    if selections['wheel']['selected'] is not None:
        parameters['wheel_speed_topic'] = selections['wheel']['selected']
    if selections['odom']['selected'] is not None:
        parameters['odom_topic'] = selections['odom']['selected']
        parameters['odom_input_mode'] = 'relative'
    if selections['initial_pose']['selected'] is not None:
        if selections['initial_pose'].get('message_type') == \
                'geometry_msgs/msg/PoseWithCovarianceStamped':
            parameters['initial_pose_covariance_topic'] = selections['initial_pose']['selected']
        else:
            parameters['initial_pose_topic'] = selections['initial_pose']['selected']
    for name, value in imu['heuristic_continuous_psd'].items():
        if value is not None and value > 0.0:
            parameters[name] = value
    return {'ekf_localization': {'ros__parameters': parameters}}


def analyze(path, topic_requests, max_samples=DEFAULT_MAX_SAMPLES_PER_TOPIC,
            gravity=GRAVITY_MPS2):
    """Inspect a bag and return a JSON/YAML-friendly report."""
    topic_types, selections, records = read_bag(path, topic_requests, max_samples)
    bag_metadata = load_bag_metadata(path)
    for role, selection in selections.items():
        selected = selection['selected']
        if selected is not None:
            selection['message_type'] = topic_types[selected]

    checks = []
    role_requirements = {
        'imu': ('error', 'missing_imu', 'No IMU topic was found.'),
        'gnss': ('warn', 'missing_gnss', 'No GNSS position topic was found.'),
        'wheel': ('warn', 'missing_wheel', 'No wheel-speed topic was found.'),
        'initial_pose': ('error', 'missing_initial_pose', 'No initial-pose topic was found.'),
    }
    for role, (severity, code, message) in role_requirements.items():
        if selections[role]['selected'] is None:
            checks.append(check(severity, code, message, selections[role]))
    for role, selection in selections.items():
        if selection['selection'] == 'requested_missing':
            checks.append(check(
                'error' if role == 'imu' else 'warn', 'requested_topic_missing',
                '{} topic was requested but not found.'.format(role), selection))

    topic_report = {}
    for name, record in records.items():
        topic_report[name] = summarize_topic(record)
        topic_report[name]['qos'] = (bag_metadata.get('topics', {}).get(name, {})
                                     .get('offered_qos_profiles', []))
        topic_report[name]['qos_available'] = bool(topic_report[name]['qos'])
        timing = topic_report[name]['timing']
        if timing['negative_or_duplicate_intervals']:
            checks.append(check(
                'error', 'non_monotonic_sensor_time',
                'Sensor timestamps are not strictly increasing on {}.'.format(name),
                {'intervals': timing['negative_or_duplicate_intervals']}))
        if record.covariance_invalid:
            checks.append(check(
                'warn', 'invalid_covariance',
                'Some covariance samples are invalid on {}.'.format(name),
                {'samples': record.covariance_invalid}))
        if len(record.frames) > 1:
            checks.append(check(
                'warn', 'multiple_frames',
                'More than one frame_id appears on {}.'.format(name),
                {'frames': sorted(record.frames)}))
        if not record.frames:
            checks.append(check(
                'warn', 'missing_frame_id',
                'No frame_id was found on {}.'.format(name)))

    imu_record = records.get(selections['imu']['selected'])
    imu = imu_statistics(imu_record, gravity) if imu_record else {
        'sample_count': 0, 'stationary_sample_count': 0, 'rate_hz': None,
        'gyro_std_radps': [None, None, None], 'accel_std_mps2': [None, None, None],
        'stationary_accel_norm_mps2': None,
        'heuristic_white_noise_density': {},
        'heuristic_continuous_psd': {},
    }
    if imu_record:
        if imu['stationary_sample_count'] < 50:
            checks.append(check(
                'warn', 'insufficient_stationary_samples',
                'Fewer than 50 stationary IMU samples were detected.',
                {'stationary_sample_count': imu['stationary_sample_count']}))
        if imu['rate_hz'] is None:
            checks.append(check('error', 'invalid_imu_timing', 'IMU rate could not be estimated.'))
        elif imu['rate_hz'] < 10.0:
            checks.append(check(
                'warn', 'low_imu_rate', 'Detected IMU rate is unusually low.',
                {'rate_hz': imu['rate_hz']}))
        if imu['stationary_accel_norm_mps2'] is not None and abs(
                imu['stationary_accel_norm_mps2'] - gravity) > 0.5:
            checks.append(check(
                'warn', 'imu_gravity_convention',
                'Stationary acceleration norm is not close to configured gravity.',
                {'norm_mps2': imu['stationary_accel_norm_mps2'], 'gravity_mps2': gravity}))

    errors = sum(item['severity'] == 'error' for item in checks)
    warnings = sum(item['severity'] == 'warn' for item in checks)
    status = 'fail' if errors else ('warn' if warnings else 'pass')
    skeleton_selections = dict(selections)
    profile = profile_skeleton(skeleton_selections, imu, gravity)
    return {
        'schema_version': 1,
        'tool': 'kf_doctor',
        'bag': {
            'path': str(Path(path).resolve()),
            'metadata': bag_metadata,
            'topics': topic_report,
        },
        'topic_types': topic_types,
        'selection': selections,
        'imu': imu,
        'checks': checks,
        'summary': {'status': status, 'errors': errors, 'warnings': warnings},
        'profile_skeleton': profile,
    }


def main(argv=None):
    """Run bag diagnostics and write a report plus optional YAML skeleton."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output-report', type=Path, required=True)
    parser.add_argument('--output-profile', type=Path)
    parser.add_argument('--imu-topic', default='')
    parser.add_argument('--gnss-topic', default='')
    parser.add_argument('--wheel-topic', default='')
    parser.add_argument('--odom-topic', default='')
    parser.add_argument('--initial-pose-topic', default='')
    parser.add_argument('--max-samples-per-topic', type=int,
                        default=DEFAULT_MAX_SAMPLES_PER_TOPIC)
    parser.add_argument('--gravity-mps2', type=float, default=GRAVITY_MPS2)
    args = parser.parse_args(argv)
    if not args.bag.is_dir():
        parser.error('--bag must be a rosbag2 directory')
    if args.max_samples_per_topic < 2:
        parser.error('--max-samples-per-topic must be at least 2')
    requests = {
        'imu': args.imu_topic,
        'gnss': args.gnss_topic,
        'wheel': args.wheel_topic,
        'odom': args.odom_topic,
        'initial_pose': args.initial_pose_topic,
    }
    try:
        report = analyze(args.bag, requests, args.max_samples_per_topic, args.gravity_mps2)
        args.output_report.parent.mkdir(parents=True, exist_ok=True)
        args.output_report.write_text(
            json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if args.output_profile:
            args.output_profile.parent.mkdir(parents=True, exist_ok=True)
            args.output_profile.write_text(
                yaml.safe_dump(report['profile_skeleton'], sort_keys=False), encoding='utf-8')
        print('{}: {}'.format(report['summary']['status'], args.output_report))
        return 1 if report['summary']['status'] == 'fail' else 0
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
