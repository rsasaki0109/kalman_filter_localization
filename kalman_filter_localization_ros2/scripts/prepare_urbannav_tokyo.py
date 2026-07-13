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
"""Convert UrbanNav Tokyo CSV and RTKLIB output into ROS 2 evaluation inputs."""

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import sys


GPS_WEEK_SECONDS = 604800.0
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def normalize_fields(row):
    """Strip the spaces present after commas in official UrbanNav CSV files."""
    return {key.strip(): value.strip() for key, value in row.items()}


def gps_stamp(week, tow):
    """Return continuous GPS seconds suitable for consistent ROS timestamps."""
    return float(week) * GPS_WEEK_SECONDS + float(tow)


def geodetic_to_ecef(latitude_deg, longitude_deg, height):
    """Convert WGS-84 geodetic coordinates to ECEF metres."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude ** 2)
    return (
        (prime_vertical + height) * math.cos(latitude) * math.cos(longitude),
        (prime_vertical + height) * math.cos(latitude) * math.sin(longitude),
        (prime_vertical * (1.0 - WGS84_E2) + height) * sin_latitude,
    )


def ecef_to_enu(ecef, origin_ecef, origin_latitude_deg, origin_longitude_deg):
    """Rotate an ECEF displacement into local east, north, up coordinates."""
    latitude = math.radians(origin_latitude_deg)
    longitude = math.radians(origin_longitude_deg)
    dx, dy, dz = (ecef[index] - origin_ecef[index] for index in range(3))
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (-math.sin(latitude) * math.cos(longitude) * dx -
             math.sin(latitude) * math.sin(longitude) * dy +
             math.cos(latitude) * dz)
    up = (math.cos(latitude) * math.cos(longitude) * dx +
          math.cos(latitude) * math.sin(longitude) * dy +
          math.sin(latitude) * dz)
    return east, north, up


def read_reference(path):
    """Read official Applanix reference CSV rows."""
    rows = []
    with open(path, newline='', encoding='utf-8-sig') as stream:
        for raw in csv.DictReader(stream, skipinitialspace=True):
            row = normalize_fields(raw)
            rows.append({
                'stamp': gps_stamp(row['GPS Week'], row['GPS TOW (s)']),
                'latitude': float(row['Latitude (deg)']),
                'longitude': float(row['Longitude (deg)']),
                'height': float(row['Ellipsoid Height (m)']),
                'heading': float(row['Heading (deg)']),
            })
    if not rows:
        raise ValueError('reference CSV is empty')
    return rows


def write_reference(path, rows, origin):
    """Write ground truth in the evaluator's local ENU CSV format."""
    origin_ecef = geodetic_to_ecef(*origin)
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp', 'x', 'y', 'z', 'yaw'))
        for row in rows:
            ecef = geodetic_to_ecef(
                row['latitude'], row['longitude'], row['height'])
            east, north, up = ecef_to_enu(ecef, origin_ecef, origin[0], origin[1])
            yaw = math.atan2(
                math.sin(math.radians(90.0 - row['heading'])),
                math.cos(math.radians(90.0 - row['heading'])))
            writer.writerow((row['stamp'], east, north, up, yaw))


def read_rtk_position(path, week):
    """Read comma-separated RTKLIB latitude/longitude/height solutions."""
    rows = []
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            if not line.strip() or line.startswith('%'):
                continue
            fields = [field.strip() for field in line.split(',')]
            timestamp = datetime.strptime(fields[0], '%Y/%m/%d %H:%M:%S.%f')
            weekday_seconds = (
                (timestamp.weekday() + 1) % 7 * 86400.0 +
                timestamp.hour * 3600.0 + timestamp.minute * 60.0 +
                timestamp.second + timestamp.microsecond * 1e-6)
            rows.append({
                'stamp': gps_stamp(week, weekday_seconds),
                'latitude': float(fields[1]),
                'longitude': float(fields[2]),
                'height': float(fields[3]),
            })
    if not rows:
        raise ValueError('RTKLIB position file is empty')
    return rows


def read_imu(path):
    """Read official TAG264 IMU CSV and convert to ROS FLU body axes."""
    rows = []
    with open(path, newline='', encoding='utf-8-sig') as stream:
        for raw in csv.DictReader(stream, skipinitialspace=True):
            row = normalize_fields(raw)
            rows.append({
                'stamp': gps_stamp(row['GPS Week'], row['GPS TOW (s)']),
                'acceleration': (
                    float(row['Acceleration X (m/s^2)']),
                    -float(row['Acceleration Y (m/s^2)']),
                    -float(row['Acceleration Z (m/s^2)'])),
                'angular_velocity': (
                    float(row['Angular rate X (rad/s)']),
                    -float(row['Angular rate Y (rad/s)']),
                    -float(row['Angular rate Z (rad/s)'])),
                'wheel_speed': float(row['Wheel velocity (m/s)']),
            })
    if not rows:
        raise ValueError('IMU CSV is empty')
    return rows


def to_ros_time(stamp):
    """Convert floating-point seconds to a builtin_interfaces Time message."""
    from builtin_interfaces.msg import Time
    seconds = math.floor(stamp)
    nanoseconds = round((stamp - seconds) * 1e9)
    if nanoseconds == 1000000000:
        seconds += 1
        nanoseconds = 0
    return Time(sec=seconds, nanosec=nanoseconds)


def write_bag(path, imu_rows, position_rows, origin, initial_yaw):
    """Write time-ordered IMU and local GNSS PoseStamped messages."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/sensing/imu/imu_data', type='sensor_msgs/msg/Imu',
        serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/gnss_pose', type='geometry_msgs/msg/PoseStamped',
        serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/ekf_localization/initial_pose',
        type='geometry_msgs/msg/PoseStamped', serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name='/wheel_speed',
        type='geometry_msgs/msg/TwistWithCovarianceStamped',
        serialization_format='cdr'))
    origin_ecef = geodetic_to_ecef(*origin)
    events = [(row['stamp'], 'imu', row) for row in imu_rows]
    events.extend((row['stamp'], 'wheel', row) for row in imu_rows)
    events.extend((row['stamp'], 'gnss', row) for row in position_rows)
    first_stamp = min(imu_rows[0]['stamp'], position_rows[0]['stamp'])
    # PoseStamped is volatile. Repeating initialization before sensor playback prevents
    # a one-shot message from being lost while rosbag2 publishers discover subscribers.
    events.extend(
        (first_stamp - 2.0 + index * 0.1, 'initial', None)
        for index in range(20))
    for stamp, kind, row in sorted(events, key=lambda event: event[0]):
        timestamp = to_ros_time(stamp)
        if kind == 'imu':
            message = Imu()
            message.header.stamp = timestamp
            message.header.frame_id = 'imu_link'
            message.orientation_covariance[0] = -1.0
            message.linear_acceleration.x, message.linear_acceleration.y, \
                message.linear_acceleration.z = row['acceleration']
            message.angular_velocity.x, message.angular_velocity.y, \
                message.angular_velocity.z = row['angular_velocity']
            topic = '/sensing/imu/imu_data'
        elif kind == 'wheel':
            message = TwistWithCovarianceStamped()
            message.header.stamp = timestamp
            message.header.frame_id = 'base_link'
            message.twist.twist.linear.x = row['wheel_speed']
            topic = '/wheel_speed'
        elif kind == 'gnss':
            message = PoseStamped()
            message.header.stamp = timestamp
            message.header.frame_id = 'map'
            ecef = geodetic_to_ecef(
                row['latitude'], row['longitude'], row['height'])
            position = ecef_to_enu(
                ecef, origin_ecef, origin[0], origin[1])
            message.pose.position.x, message.pose.position.y, \
                message.pose.position.z = position
            message.pose.orientation.w = 1.0
            topic = '/gnss_pose'
        else:
            message = PoseStamped()
            message.header.stamp = timestamp
            message.header.frame_id = 'map'
            message.pose.orientation.z = math.sin(initial_yaw / 2.0)
            message.pose.orientation.w = math.cos(initial_yaw / 2.0)
            topic = '/ekf_localization/initial_pose'
        writer.write(topic, serialize_message(message), round(stamp * 1e9))


def main(argv=None):
    """Prepare one official UrbanNav Tokyo sequence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--imu-csv', type=Path, required=True)
    parser.add_argument('--reference-csv', type=Path, required=True)
    parser.add_argument('--rtklib-pos', type=Path, required=True)
    parser.add_argument('--output-bag', type=Path, required=True)
    parser.add_argument('--output-reference-csv', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        for path in (args.imu_csv, args.reference_csv, args.rtklib_pos):
            if not path.is_file():
                raise ValueError('input file does not exist: {}'.format(path))
        if args.output_bag.exists():
            raise ValueError('output bag already exists: {}'.format(args.output_bag))
        reference = read_reference(args.reference_csv)
        origin = (
            reference[0]['latitude'], reference[0]['longitude'],
            reference[0]['height'])
        week = math.floor(reference[0]['stamp'] / GPS_WEEK_SECONDS)
        imu = read_imu(args.imu_csv)
        positions = read_rtk_position(args.rtklib_pos, week)
        args.output_bag.parent.mkdir(parents=True, exist_ok=True)
        args.output_reference_csv.parent.mkdir(parents=True, exist_ok=True)
        write_reference(args.output_reference_csv, reference, origin)
        initial_yaw = math.radians(90.0 - reference[0]['heading'])
        write_bag(args.output_bag, imu, positions, origin, initial_yaw)
        print('wrote {} IMU and {} GNSS samples'.format(len(imu), len(positions)))
        return 0
    except (ImportError, OSError, ValueError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
