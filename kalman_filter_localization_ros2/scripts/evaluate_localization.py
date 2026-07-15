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
import statistics
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


def select_time_range(samples, start_stamp=None, end_stamp=None):
    """Select an inclusive timestamp range without changing sample order."""
    if start_stamp is not None and end_stamp is not None and start_stamp > end_stamp:
        raise ValueError('--start-stamp must not exceed --end-stamp')
    return [sample for sample in samples
            if (start_stamp is None or sample.stamp >= start_stamp)
            and (end_stamp is None or sample.stamp <= end_stamp)]


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


def summarize_error_values(values):
    """Return stable scalar statistics, or None fields for an empty series."""
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return {'samples': 0, 'mean': None, 'median': None, 'rmse': None, 'max': None}
    return {
        'samples': len(finite),
        'mean': statistics.mean(finite),
        'median': statistics.median(finite),
        'rmse': math.sqrt(statistics.mean(value * value for value in finite)),
        'max': max(finite),
    }


def relative_pose_error(matches, time_horizons=(), distance_horizons=()):
    """Compute translation/yaw RPE at requested elapsed-time and reference-distance horizons."""
    if len(matches) < 2:
        return {'time': {}, 'distance': {}}
    cumulative_distance = [0.0]
    for before, after in zip(matches, matches[1:]):
        cumulative_distance.append(cumulative_distance[-1] + math.sqrt(
            (after['rx'] - before['rx']) ** 2 +
            (after['ry'] - before['ry']) ** 2 +
            (after['rz'] - before['rz']) ** 2))

    def metrics(pairs):
        translation, horizontal, yaw = [], [], []
        for first, second in pairs:
            edx = second['ex'] - first['ex']
            edy = second['ey'] - first['ey']
            edz = second['ez'] - first['ez']
            rdx = second['rx'] - first['rx']
            rdy = second['ry'] - first['ry']
            rdz = second['rz'] - first['rz']
            dx, dy, dz = edx - rdx, edy - rdy, edz - rdz
            translation.append(math.sqrt(dx * dx + dy * dy + dz * dz))
            horizontal.append(math.hypot(dx, dy))
            yaw.append(math.degrees(normalize_angle(
                (second['eyaw'] - first['eyaw']) -
                (second['ryaw'] - first['ryaw']))))
        return {
            'translation_m': summarize_error_values(translation),
            'horizontal_m': summarize_error_values(horizontal),
            'yaw_deg': summarize_error_values(yaw),
        }

    result = {'time': {}, 'distance': {}}
    stamps = [match['stamp'] for match in matches]
    for horizon in time_horizons:
        pairs = []
        for index, match in enumerate(matches[:-1]):
            target = match['stamp'] + horizon
            right = bisect_left(stamps, target, lo=index + 1)
            if right < len(matches):
                pairs.append((match, matches[right]))
        result['time'][str(horizon)] = metrics(pairs)
    for horizon in distance_horizons:
        pairs = []
        for index, match in enumerate(matches[:-1]):
            target = cumulative_distance[index] + horizon
            right = bisect_left(cumulative_distance, target, lo=index + 1)
            if right < len(matches):
                pairs.append((match, matches[right]))
        result['distance'][str(horizon)] = metrics(pairs)
    return result


def evaluate_outages(errors, segments, settling_threshold_m=1.0, settling_window_sec=1.0):
    """Report endpoint drift and post-outage overshoot/settling for named time segments."""
    result = []
    for segment in segments or []:
        start, end = float(segment['start']), float(segment['end'])
        before = [row for row in errors if row['stamp'] <= start]
        during = [row for row in errors if start <= row['stamp'] <= end]
        after = [row for row in errors if row['stamp'] >= end]
        start_error = before[-1]['error_3d'] if before else None
        endpoint_error = during[-1]['error_3d'] if during else None
        overshoot = max((row['error_3d'] for row in after), default=None)
        settling_time = None
        for row in after:
            window_end = row['stamp'] + settling_window_sec
            window = [candidate for candidate in after
                      if row['stamp'] <= candidate['stamp'] <= window_end]
            if window and window[-1]['stamp'] >= window_end and all(
                    candidate['error_3d'] <= settling_threshold_m for candidate in window):
                settling_time = row['stamp'] - end
                break
        result.append({
            'name': segment.get('name', 'outage'), 'start': start, 'end': end,
            'start_error_3d_m': start_error, 'endpoint_error_3d_m': endpoint_error,
            'endpoint_drift_growth_m': (
                endpoint_error - start_error
                if endpoint_error is not None and start_error is not None else None),
            'reacquisition_overshoot_3d_m': overshoot,
            'reacquisition_settling_time_sec': settling_time,
            'settling_threshold_m': settling_threshold_m,
            'settling_window_sec': settling_window_sec,
        })
    return result


def evaluate(
        estimates, references, max_gap, time_offset=0.0,
        align_translation=False, start_stamp=None, end_stamp=None,
        rpe_time_horizons=(), rpe_distance_horizons=(), outage_segments=None,
        settling_threshold_m=1.0, settling_window_sec=1.0):
    estimates = select_time_range(estimates, start_stamp, end_stamp)
    if not estimates or not references:
        raise ValueError('estimate and reference trajectories must not be empty')
    reference_stamps = [sample.stamp for sample in references]
    errors = []
    matches = []
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
        matches.append({
            'stamp': estimate.stamp,
            'ex': estimate.x, 'ey': estimate.y, 'ez': estimate.z, 'eyaw': estimate.yaw,
            'rx': reference.x, 'ry': reference.y, 'rz': reference.z, 'ryaw': reference.yaw,
        })
    if not errors:
        raise ValueError('no overlapping samples; check timestamps and --max-reference-gap')

    alignment = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    if align_translation:
        alignment = {
            axis: statistics.median(row['d' + axis] for row in errors)
            for axis in ('x', 'y', 'z')}
        for row in errors:
            row['dx'] -= alignment['x']
            row['dy'] -= alignment['y']
            row['dz'] -= alignment['z']
            row['horizontal'] = math.hypot(row['dx'], row['dy'])
            row['error_3d'] = math.sqrt(
                row['dx'] ** 2 + row['dy'] ** 2 + row['dz'] ** 2)

    def rmse(key):
        values = [row[key] for row in errors if math.isfinite(row[key])]
        return math.sqrt(sum(value * value for value in values) / len(values)) if values else None

    ape = {
        'translation_3d_m': summarize_error_values(row['error_3d'] for row in errors),
        'horizontal_m': summarize_error_values(row['horizontal'] for row in errors),
        'vertical_m': summarize_error_values(abs(row['dz']) for row in errors),
        'yaw_deg': summarize_error_values(abs(row['yaw_deg']) for row in errors),
    }
    policy = {
        'alignment': 'median_translation' if align_translation else 'none',
        'interpolation_tolerance_sec': max_gap,
        'time_offset_sec': time_offset,
        'start_stamp': start_stamp,
        'end_stamp': end_stamp,
    }
    return {'matched_samples': len(errors), 'estimate_samples': len(estimates),
            'match_ratio': len(errors) / len(estimates),
            'missing_ratio': 1.0 - len(errors) / len(estimates),
            'duration_sec': errors[-1]['stamp'] - errors[0]['stamp'],
            'translation_alignment_m': alignment,
            'rmse_3d_m': rmse('error_3d'), 'rmse_horizontal_m': rmse('horizontal'),
            'rmse_vertical_m': rmse('dz'), 'yaw_rmse_deg': rmse('yaw_deg'),
            'max_error_3d_m': max(row['error_3d'] for row in errors),
            'ape': ape,
            'rpe': relative_pose_error(matches, rpe_time_horizons, rpe_distance_horizons),
            'outages': evaluate_outages(
                errors, outage_segments, settling_threshold_m, settling_window_sec),
            'evaluation_policy': policy}, errors


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


def parse_outage_segment(value):
    """Parse NAME,START,END into a manifest-ready outage definition."""
    fields = value.split(',')
    if len(fields) != 3:
        raise argparse.ArgumentTypeError('outage segment must be NAME,START,END')
    try:
        start, end = float(fields[1]), float(fields[2])
    except ValueError as error:
        raise argparse.ArgumentTypeError('outage timestamps must be numeric') from error
    if not math.isfinite(start) or not math.isfinite(end) or start >= end:
        raise argparse.ArgumentTypeError('outage START must be finite and below END')
    return {'name': fields[0], 'start': start, 'end': end}


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
    parser.add_argument(
        '--align-translation', action='store_true',
        help='remove median XYZ offset before calculating position errors')
    parser.add_argument('--start-stamp', type=float, help='inclusive estimate start timestamp')
    parser.add_argument('--end-stamp', type=float, help='inclusive estimate end timestamp')
    parser.add_argument('--rpe-time-sec', type=float, action='append', default=[])
    parser.add_argument('--rpe-distance-m', type=float, action='append', default=[])
    parser.add_argument('--outage-segment', type=parse_outage_segment, action='append', default=[])
    parser.add_argument('--settling-threshold-m', type=float, default=1.0)
    parser.add_argument('--settling-window-sec', type=float, default=1.0)
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
        summary, errors = evaluate(
            estimates, references, args.max_reference_gap, args.time_offset,
            args.align_translation, args.start_stamp, args.end_stamp,
            args.rpe_time_sec, args.rpe_distance_m, args.outage_segment,
            args.settling_threshold_m, args.settling_window_sec)
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
