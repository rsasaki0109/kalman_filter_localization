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
"""Generate a deterministic stationary rosbag2 and matching reference CSV."""

import argparse
import csv
from pathlib import Path


def ros_time(seconds):
    """Convert floating-point seconds to builtin_interfaces/Time."""
    from builtin_interfaces.msg import Time

    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1.0e9))
    if nanoseconds == 1_000_000_000:
        whole += 1
        nanoseconds = 0
    return Time(sec=whole, nanosec=nanoseconds)


def pose(stamp):
    """Build an identity PoseStamped at the requested time."""
    from geometry_msgs.msg import PoseStamped

    message = PoseStamped()
    message.header.stamp = ros_time(stamp)
    message.header.frame_id = 'map'
    message.pose.orientation.w = 1.0
    return message


def write_fixture(bag_path, reference_path):
    """Write initialization, stationary IMU, GNSS, and reference samples."""
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    topics = (
        ('/ekf_localization/initial_pose', 'geometry_msgs/msg/PoseStamped'),
        ('/gnss_pose', 'geometry_msgs/msg/PoseStamped'),
        ('/sensing/imu/imu_data', 'sensor_msgs/msg/Imu'),
    )
    for name, message_type in topics:
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=name, type=message_type, serialization_format='cdr'))

    start = 10.0
    events = []
    for index in range(20):
        stamp = start - 2.0 + 0.1 * index
        events.append((stamp, '/ekf_localization/initial_pose', pose(stamp)))
    reference_rows = []
    for index in range(101):
        stamp = start + 0.01 * index
        message = Imu()
        message.header.stamp = ros_time(stamp)
        message.header.frame_id = 'imu_link'
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.z = 9.80665
        events.append((stamp, '/sensing/imu/imu_data', message))
        reference_rows.append((stamp, 0.0, 0.0, 0.0, 0.0))
    for index in range(11):
        stamp = start + 0.1 * index
        events.append((stamp, '/gnss_pose', pose(stamp)))

    for stamp, topic, message in sorted(events, key=lambda item: (item[0], item[1])):
        writer.write(topic, serialize_message(message), int(round(stamp * 1.0e9)))

    with reference_path.open('w', encoding='utf-8', newline='') as stream:
        output = csv.writer(stream)
        output.writerow(('stamp', 'x', 'y', 'z', 'yaw'))
        output.writerows(reference_rows)


def main(argv=None):
    """Parse command-line paths and generate the fixture."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--reference-csv', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.bag.exists() or args.reference_csv.exists():
        parser.error('output paths must not exist')
    args.bag.parent.mkdir(parents=True, exist_ok=True)
    args.reference_csv.parent.mkdir(parents=True, exist_ok=True)
    write_fixture(args.bag, args.reference_csv)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
