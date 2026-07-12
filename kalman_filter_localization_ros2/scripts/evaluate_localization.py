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
"""Evaluate a localization trajectory against a timestamped reference trajectory."""

import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
import json
import math
import sys


@dataclass(frozen=True)
class Sample:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_csv(path):
    samples = []
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        missing = {'stamp', 'x', 'y', 'z'}.difference(reader.fieldnames or [])
        if missing:
            raise ValueError('{} is missing columns: {}'.format(path, ', '.join(sorted(missing))))
        for row_number, row in enumerate(reader, start=2):
            try:
                if row.get('yaw') not in (None, ''):
                    yaw = float(row['yaw'])
                elif all(row.get(key) not in (None, '') for key in ('qx', 'qy', 'qz', 'qw')):
                    yaw = quaternion_yaw(*(float(row[key]) for key in ('qx', 'qy', 'qz', 'qw')))
                else:
                    yaw = float('nan')
                samples.append(Sample(float(row['stamp']), float(row['x']), float(row['y']),
                                      float(row['z']), yaw))
            except ValueError as error:
                message = '{}:{} contains a non-numeric value'.format(path, row_number)
                raise ValueError(message) from error
    return sorted(samples, key=lambda sample: sample.stamp)


def read_bag(path, topic):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError('rosbag input requires a sourced ROS 2 environment') from error

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError('topic {!r} not found in {}'.format(topic, path))
    message_type = get_message(topic_types[topic])
    samples = []
    while reader.has_next():
        current_topic, data, recorded_stamp = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(data, message_type)
        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        header = getattr(message, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        else:
            stamp = recorded_stamp * 1e-9
        samples.append(Sample(stamp, pose.position.x, pose.position.y, pose.position.z,
                              quaternion_yaw(pose.orientation.x, pose.orientation.y,
                                             pose.orientation.z, pose.orientation.w)))
    return sorted(samples, key=lambda sample: sample.stamp)


def interpolate(samples, stamps, stamp, max_gap):
    right = bisect_left(stamps, stamp)
    if right < len(samples) and abs(samples[right].stamp - stamp) <= 1e-9:
        return samples[right]
    if right == 0 or right == len(samples):
        return None
    before, after = samples[right - 1], samples[right]
    if after.stamp - before.stamp > max_gap:
        return None
    ratio = (stamp - before.stamp) / (after.stamp - before.stamp)
    yaw_delta = normalize_angle(after.yaw - before.yaw)
    yaw = before.yaw + ratio * yaw_delta if math.isfinite(yaw_delta) else float('nan')
    return Sample(
        stamp,
        before.x + ratio * (after.x - before.x),
        before.y + ratio * (after.y - before.y),
        before.z + ratio * (after.z - before.z),
        yaw)


def evaluate(estimates, references, max_gap, time_offset=0.0):
    if not estimates or not references:
        raise ValueError('estimate and reference trajectories must not be empty')
    reference_stamps = [sample.stamp for sample in references]
    errors = []
    for estimate in estimates:
        reference = interpolate(
            references, reference_stamps, estimate.stamp + time_offset, max_gap)
        if reference is None:
            continue
        dx, dy, dz = estimate.x - reference.x, estimate.y - reference.y, estimate.z - reference.z
        yaw_error = normalize_angle(estimate.yaw - reference.yaw)
        errors.append({
            'stamp': estimate.stamp,
            'dx': dx,
            'dy': dy,
            'dz': dz,
            'horizontal': math.hypot(dx, dy),
            'error_3d': math.sqrt(dx * dx + dy * dy + dz * dz),
            'yaw_deg': (
                math.degrees(yaw_error) if math.isfinite(yaw_error)
                else float('nan')),
        })
    if not errors:
        raise ValueError('no overlapping samples; check timestamps and --max-reference-gap')

    def rmse(key):
        values = [row[key] for row in errors if math.isfinite(row[key])]
        return math.sqrt(sum(value * value for value in values) / len(values)) if values else None

    return {'matched_samples': len(errors), 'estimate_samples': len(estimates),
            'match_ratio': len(errors) / len(estimates),
            'duration_sec': errors[-1]['stamp'] - errors[0]['stamp'],
            'rmse_3d_m': rmse('error_3d'), 'rmse_horizontal_m': rmse('horizontal'),
            'rmse_vertical_m': rmse('dz'), 'yaw_rmse_deg': rmse('yaw_deg'),
            'max_error_3d_m': max(row['error_3d'] for row in errors)}, errors


def write_errors(path, errors):
    fields = ['stamp', 'dx', 'dy', 'dz', 'horizontal', 'error_3d', 'yaw_deg']
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(errors)


def check_thresholds(summary, thresholds):
    """Return human-readable failures for configured metric thresholds."""
    failures = []
    for metric, limit in thresholds.items():
        if limit is None:
            continue
        value = summary[metric]
        if value is None:
            failures.append('{} is unavailable'.format(metric))
        elif metric == 'match_ratio':
            if value < limit:
                failures.append('{} {:.6g} is below {:.6g}'.format(metric, value, limit))
        elif value > limit:
            failures.append('{} {:.6g} exceeds {:.6g}'.format(metric, value, limit))
    return failures


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--estimate-csv')
    source.add_argument('--bag', help='rosbag2 directory containing both trajectories')
    parser.add_argument('--reference-csv')
    parser.add_argument('--estimate-topic', default='/current_pose')
    parser.add_argument('--reference-topic', default='/ground_truth/pose')
    parser.add_argument('--max-reference-gap', type=float, default=0.2)
    parser.add_argument('--time-offset', type=float, default=0.0,
                        help='seconds added to estimate timestamps before matching')
    parser.add_argument('--output-json')
    parser.add_argument('--output-csv', help='write per-sample errors')
    parser.add_argument('--max-rmse-3d', type=float)
    parser.add_argument('--max-rmse-horizontal', type=float)
    parser.add_argument('--max-rmse-vertical', type=float)
    parser.add_argument('--max-yaw-rmse-deg', type=float)
    parser.add_argument('--max-error-3d', type=float)
    parser.add_argument('--min-match-ratio', type=float)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.bag:
            estimates = read_bag(args.bag, args.estimate_topic)
            references = (
                read_csv(args.reference_csv) if args.reference_csv
                else read_bag(args.bag, args.reference_topic))
        else:
            if not args.reference_csv:
                raise ValueError('--reference-csv is required with --estimate-csv')
            estimates, references = read_csv(args.estimate_csv), read_csv(args.reference_csv)
        summary, errors = evaluate(estimates, references, args.max_reference_gap, args.time_offset)
        thresholds = {
            'rmse_3d_m': args.max_rmse_3d,
            'rmse_horizontal_m': args.max_rmse_horizontal,
            'rmse_vertical_m': args.max_rmse_vertical,
            'yaw_rmse_deg': args.max_yaw_rmse_deg,
            'max_error_3d_m': args.max_error_3d,
            'match_ratio': args.min_match_ratio,
        }
        failures = check_thresholds(summary, thresholds)
        summary['passed'] = not failures
        summary['threshold_failures'] = failures
        rendered = json.dumps(summary, indent=2, sort_keys=True)
        print(rendered)
        if args.output_json:
            with open(args.output_json, 'w', encoding='utf-8') as stream:
                stream.write(rendered + '\n')
        if args.output_csv:
            write_errors(args.output_csv, errors)
        if failures:
            for failure in failures:
                print('threshold failure: {}'.format(failure), file=sys.stderr)
            return 1
        return 0
    except (OSError, RuntimeError, ValueError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
