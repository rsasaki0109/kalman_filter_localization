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
"""Convert an Applanix/vehicle-speed rosbag2 into an open-sky evaluation bag."""

import argparse
import csv
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec
from importlib.util import spec_from_loader
import math
from pathlib import Path
import sqlite3
import struct
import sys

APPLANIX_TOPIC = '/lvx_client/gsof/ins_solution_49'
WHEEL_TOPIC = '/vehicle/status/velocity_status'


def load_urbannav_helpers():
    """Load shared geodesy helpers in source and installed layouts."""
    directory = Path(__file__).resolve().parent
    source_path = directory / 'prepare_urbannav_tokyo.py'
    path = source_path if source_path.exists() else directory / 'prepare_urbannav_tokyo'
    loader = SourceFileLoader('prepare_urbannav_tokyo', str(path))
    spec = spec_from_loader(loader.name, loader)
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


HELPERS = load_urbannav_helpers()
ecef_to_enu = HELPERS.ecef_to_enu
geodetic_to_ecef = HELPERS.geodetic_to_ecef
to_ros_time = HELPERS.to_ros_time


def quaternion_from_rpy(roll, pitch, yaw):
    """Return an XYZW quaternion for intrinsic roll, pitch, and yaw."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def applanix_yaw(heading_deg):
    """Convert north-clockwise heading to ENU counter-clockwise yaw."""
    return math.atan2(
        math.sin(math.radians(90.0 - heading_deg)),
        math.cos(math.radians(90.0 - heading_deg)))


def applanix_acceleration_to_specific_force(
        acceleration_long, acceleration_transverse, acceleration_down,
        roll_deg, pitch_deg, gravity_mps2=9.80665,
        gravity_compensated=True):
    """Convert GSOF FRD acceleration to ROS FLU specific force."""
    dynamic_acceleration = (
        acceleration_long, -acceleration_transverse, -acceleration_down)
    if not gravity_compensated:
        return dynamic_acceleration
    roll = math.radians(roll_deg)
    pitch = -math.radians(pitch_deg)
    gravity_body = (
        -gravity_mps2 * math.sin(pitch),
        gravity_mps2 * math.cos(pitch) * math.sin(roll),
        gravity_mps2 * math.cos(pitch) * math.cos(roll),
    )
    return tuple(dynamic_acceleration[index] + gravity_body[index]
                 for index in range(3))


def decode_velocity_report(data):
    """Decode the standard VelocityReport header and three float32 fields."""
    if len(data) < 28:
        raise ValueError('VelocityReport CDR payload is too short')
    endian = '<' if data[1] == 1 else '>'
    string_length = struct.unpack_from(endian + 'I', data, 12)[0]
    offset = 16 + string_length
    offset = (offset + 3) & ~3
    if offset + 12 > len(data):
        raise ValueError('VelocityReport CDR payload has invalid string length')
    longitudinal, lateral, heading_rate = struct.unpack_from(
        endian + 'fff', data, offset)
    sec, nanosec = struct.unpack_from(endian + 'iI', data, 4)
    return sec + nanosec * 1.0e-9, longitudinal, lateral, heading_rate


def read_source(path):
    """Read Applanix truth and optional vehicle speed directly from sqlite3."""
    try:
        from applanix_msgs.msg import NavigationSolutionGsof49
        from rclpy.serialization import deserialize_message
    except ImportError as error:
        raise RuntimeError(
            'source an environment containing applanix_msgs before running') from error
    connection = sqlite3.connect(path)
    topics = {name: topic_id for topic_id, name in connection.execute(
        'SELECT id, name FROM topics')}
    missing = {APPLANIX_TOPIC}.difference(topics)
    if missing:
        raise ValueError('input bag is missing topics: {}'.format(', '.join(sorted(missing))))
    applanix = []
    query = 'SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp'
    for recorded_stamp, data in connection.execute(query, (topics[APPLANIX_TOPIC],)):
        message = deserialize_message(data, NavigationSolutionGsof49)
        # The Applanix header uses GPS week time while the vehicle message and
        # rosbag use Unix time. Use the common recording clock for all topics.
        stamp = recorded_stamp * 1.0e-9
        applanix.append((stamp, message))
    wheels = []
    if WHEEL_TOPIC in topics:
        for recorded_stamp, data in connection.execute(query, (topics[WHEEL_TOPIC],)):
            unused_header_stamp, longitudinal, lateral, heading_rate = \
                decode_velocity_report(data)
            wheels.append((recorded_stamp * 1.0e-9, longitudinal, lateral, heading_rate))
    connection.close()
    if not applanix:
        raise ValueError('Applanix input topic contains no samples')
    return applanix, wheels


def local_position(message, origin, origin_ecef):
    """Convert one Applanix LLA solution to the shared local ENU frame."""
    return ecef_to_enu(
        geodetic_to_ecef(
            message.lla.latitude, message.lla.longitude, message.lla.altitude),
        origin_ecef, origin[0], origin[1])


def write_reference(path, samples, origin):
    """Write the full-rate Applanix trajectory as evaluator CSV."""
    origin_ecef = geodetic_to_ecef(*origin)
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp', 'x', 'y', 'z', 'yaw'))
        for stamp, message in samples:
            writer.writerow((stamp, *local_position(message, origin, origin_ecef),
                             applanix_yaw(message.heading)))


def write_bag(
        path, applanix, wheels, origin, gnss_period,
        acceleration_is_gravity_compensated=True):
    """Write standard IMU, wheel speed, downsampled GNSS, and initial pose topics."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic_types = {
        '/sensing/imu/imu_data': 'sensor_msgs/msg/Imu',
        '/wheel_speed': 'geometry_msgs/msg/TwistWithCovarianceStamped',
        '/gnss_pose': 'geometry_msgs/msg/PoseStamped',
        '/ekf_localization/initial_pose': 'geometry_msgs/msg/PoseStamped',
    }
    for name, message_type in topic_types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=message_type, serialization_format='cdr'))
    origin_ecef = geodetic_to_ecef(*origin)
    events = [(stamp, 'imu', message) for stamp, message in applanix]
    events.extend((stamp, 'wheel', speed) for stamp, speed, unused_lat, unused_rate in wheels)
    next_gnss_stamp = -math.inf
    for stamp, message in applanix:
        if stamp >= next_gnss_stamp:
            events.append((stamp, 'gnss', message))
            next_gnss_stamp = stamp + gnss_period
    first_stamp, first_message = applanix[0]
    events.extend((first_stamp - 3.0 + index * 0.1, 'initial', first_message)
                  for index in range(30))
    degree_to_radian = math.pi / 180.0
    for stamp, kind, value in sorted(events, key=lambda event: event[0]):
        ros_stamp = to_ros_time(stamp)
        if kind == 'imu':
            message = Imu()
            message.header.stamp = ros_stamp
            message.header.frame_id = 'imu_link'
            message.orientation_covariance[0] = -1.0
            message.angular_velocity.x = value.ang_rate_long * degree_to_radian
            message.angular_velocity.y = -value.ang_rate_trans * degree_to_radian
            message.angular_velocity.z = -value.ang_rate_down * degree_to_radian
            acceleration = applanix_acceleration_to_specific_force(
                value.acc_long, value.acc_trans, value.acc_down,
                value.roll, value.pitch,
                gravity_compensated=acceleration_is_gravity_compensated)
            message.linear_acceleration.x, message.linear_acceleration.y, \
                message.linear_acceleration.z = acceleration
            topic = '/sensing/imu/imu_data'
        elif kind == 'wheel':
            message = TwistWithCovarianceStamped()
            message.header.stamp = ros_stamp
            message.header.frame_id = 'base_link'
            message.twist.twist.linear.x = value
            topic = '/wheel_speed'
        else:
            message = PoseStamped()
            message.header.stamp = ros_stamp
            message.header.frame_id = 'map'
            if kind == 'gnss':
                position = local_position(value, origin, origin_ecef)
                message.pose.position.x, message.pose.position.y, \
                    message.pose.position.z = position
                message.pose.orientation.w = 1.0
                topic = '/gnss_pose'
            else:
                yaw = applanix_yaw(value.heading)
                roll = value.roll * degree_to_radian
                pitch = -value.pitch * degree_to_radian
                quaternion = quaternion_from_rpy(roll, pitch, yaw)
                message.pose.orientation.x, message.pose.orientation.y, \
                    message.pose.orientation.z, message.pose.orientation.w = quaternion
                topic = '/ekf_localization/initial_pose'
        writer.write(topic, serialize_message(message), round(stamp * 1.0e9))


def main(argv=None):
    """Prepare the Applanix open-sky dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-db3', type=Path, required=True)
    parser.add_argument('--output-bag', type=Path, required=True)
    parser.add_argument('--output-reference-csv', type=Path, required=True)
    parser.add_argument('--gnss-rate-hz', type=float, default=5.0)
    parser.add_argument(
        '--wheel-time-offset-sec', type=float, default=0.0,
        help='seconds added to recorded wheel timestamps for sensor synchronization')
    parser.add_argument(
        '--acceleration-mode', choices=('gravity-compensated', 'specific-force'),
        default='gravity-compensated',
        help='interpretation of GSOF longitudinal/transverse/down acceleration')
    args = parser.parse_args(argv)
    try:
        if not args.input_db3.is_file():
            raise ValueError('input db3 does not exist: {}'.format(args.input_db3))
        if args.output_bag.exists():
            raise ValueError('output bag already exists: {}'.format(args.output_bag))
        if args.gnss_rate_hz <= 0.0:
            raise ValueError('--gnss-rate-hz must be positive')
        applanix, wheels = read_source(args.input_db3)
        wheels = [(stamp + args.wheel_time_offset_sec, speed, lateral, heading_rate)
                  for stamp, speed, lateral, heading_rate in wheels]
        first = applanix[0][1]
        origin = (first.lla.latitude, first.lla.longitude, first.lla.altitude)
        args.output_bag.parent.mkdir(parents=True, exist_ok=True)
        args.output_reference_csv.parent.mkdir(parents=True, exist_ok=True)
        write_reference(args.output_reference_csv, applanix, origin)
        write_bag(
            args.output_bag, applanix, wheels, origin, 1.0 / args.gnss_rate_hz,
            args.acceleration_mode == 'gravity-compensated')
        print('wrote {} IMU, {} wheel, and reference samples'.format(
            len(applanix), len(wheels)))
        return 0
    except (ImportError, OSError, RuntimeError, sqlite3.Error, ValueError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
