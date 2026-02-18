#!/usr/bin/env python3
"""Convert Applanix GSOF49 INS message to Imu in a ROS-like ENU+FLU convention.

This is useful when you want to fuse INS-derived attitude from
`/lvx_client/gsof/ins_solution_49` directly as the EKF IMU orientation source.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

import rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from applanix_msgs.msg import NavigationSolutionGsof49
except ImportError:  # pragma: no cover
    NavigationSolutionGsof49 = None  # type: ignore[assignment]


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    half_r = roll * 0.5
    half_p = pitch * 0.5
    half_y = yaw * 0.5
    cr = math.cos(half_r)
    sr = math.sin(half_r)
    cp = math.cos(half_p)
    sp = math.sin(half_p)
    cy = math.cos(half_y)
    sy = math.sin(half_y)
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = cy * sp * cr + sy * cp * sr
    qz = sy * cp * cr - cy * sp * sr
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if not (norm > 0.0) or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return qx * inv, qy * inv, qz * inv, qw * inv


def quat_to_rot(qx: float, qy: float, qz: float, qw: float):
    return [
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ],
        [
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ],
        [
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
    ]


def rot_to_quat(rot):
    r00, r01, r02 = rot[0]
    r10, r11, r12 = rot[1]
    r20, r21, r22 = rot[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not (norm > 0.0) or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    inv = 1.0 / norm
    return qx * inv, qy * inv, qz * inv, qw * inv


def matmul(a, b):
    out = []
    for i in range(3):
        out.append(
            [
                a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
                for j in range(3)
            ]
        )
    return out


def transform_body_to_ros(qx, qy, qz, qw, *, convert: bool) -> tuple[float, float, float, float]:
    if not convert:
        return qx, qy, qz, qw
    r = quat_to_rot(qx, qy, qz, qw)
    enu2ned = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    applanix2ros = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    enu2ned_r = matmul(enu2ned, r)
    corr = matmul(enu2ned_r, applanix2ros)
    return rot_to_quat(corr)


@dataclass
class OutputConfig:
    input_topic: str
    output_topic: str
    output_frame_id: str
    qos_depth: int
    output_mode: str


class Gsof49ToImu(Node):
    def __init__(self, config: OutputConfig):
        super().__init__("gsof49_to_imu")
        self._config = config
        qos = QoSProfile(depth=config.qos_depth)
        self._pub = self.create_publisher(Imu, config.output_topic, qos)
        self.create_subscription(
            NavigationSolutionGsof49,
            config.input_topic,
            self._callback,
            qos,
        )
        self.get_logger().info(
            "convert GSOF49 -> Imu "
            f"topic={config.output_topic} frame={config.output_frame_id} mode={config.output_mode}"
        )

    def _callback(self, msg: NavigationSolutionGsof49) -> None:
        def _read(msg_obj: object, *names: str) -> float:
            for name in names:
                cur = msg_obj
                ok = True
                for item in name.split("."):
                    if not hasattr(cur, item):
                        ok = False
                        break
                    cur = getattr(cur, item)
                if ok:
                    return float(cur)
            raise RuntimeError(f"Unsupported GSOF49 field; checked {names}")

        roll_rad = math.radians(_read(msg, "roll", "attitude.roll"))
        pitch_rad = math.radians(_read(msg, "pitch", "attitude.pitch"))
        yaw_rad = math.radians(_read(msg, "heading", "attitude.heading", "attitude.yaw"))
        qx, qy, qz, qw = quat_from_rpy(roll_rad, pitch_rad, yaw_rad)
        qx, qy, qz, qw = transform_body_to_ros(
            qx, qy, qz, qw, convert=self._config.output_mode == "ros"
        )

        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.frame_id = self._config.output_frame_id

        if self._config.output_mode == "identity":
            imu_msg.orientation.w = 1.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.x = 0.0
            imu_msg.linear_acceleration.z = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.x = 0.0
        else:
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            if self._config.output_mode == "ros":
                imu_msg.angular_velocity.x = math.radians(
                    _read(msg, "ang_rate_long", "angular_rate.roll", "x_rate", "x")
                )
                imu_msg.angular_velocity.y = -math.radians(
                    _read(msg, "ang_rate_trans", "angular_rate.pitch", "y_rate", "y")
                )
                imu_msg.angular_velocity.z = -math.radians(
                    _read(msg, "ang_rate_down", "angular_rate.heading", "angular_rate.yaw", "z_rate", "z")
                )
                imu_msg.linear_acceleration.x = _read(
                    msg, "acc_long", "acceleration.x", "accel.x"
                )
                imu_msg.linear_acceleration.y = -_read(
                    msg, "acc_trans", "acceleration.y", "accel.y"
                )
                imu_msg.linear_acceleration.z = -_read(
                    msg, "acc_down", "acceleration.z", "accel.z"
                )
            else:
                imu_msg.angular_velocity.x = math.radians(
                    _read(msg, "ang_rate_long", "angular_rate.roll", "x_rate", "x")
                )
                imu_msg.angular_velocity.y = math.radians(
                    _read(msg, "ang_rate_trans", "angular_rate.pitch", "y_rate", "y")
                )
                imu_msg.angular_velocity.z = math.radians(
                    _read(msg, "ang_rate_down", "angular_rate.heading", "angular_rate.yaw", "z_rate", "z")
                )
                imu_msg.linear_acceleration.x = _read(
                    msg, "acc_long", "acceleration.x", "accel.x"
                )
                imu_msg.linear_acceleration.y = _read(
                    msg, "acc_trans", "acceleration.y", "accel.y"
                )
                imu_msg.linear_acceleration.z = _read(
                    msg, "acc_down", "acceleration.z", "accel.z"
                )

        imu_msg.orientation_covariance[0] = float("nan")
        imu_msg.orientation_covariance[4] = float("nan")
        imu_msg.orientation_covariance[8] = float("nan")
        imu_msg.angular_velocity_covariance[0] = float("nan")
        imu_msg.angular_velocity_covariance[4] = float("nan")
        imu_msg.angular_velocity_covariance[8] = float("nan")
        imu_msg.linear_acceleration_covariance[0] = float("nan")
        imu_msg.linear_acceleration_covariance[4] = float("nan")
        imu_msg.linear_acceleration_covariance[8] = float("nan")
        self._pub.publish(imu_msg)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-topic", default="/ins_imu")
    p.add_argument("--output-frame-id", default="base_link")
    p.add_argument("--qos-depth", type=int, default=10)
    p.add_argument(
        "--output-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help="raw_rpy: raw roll/pitch/heading, ros: NED->ENU + FRD->FLU",
    )
    p.add_argument(
        "--input-topic",
        default="/lvx_client/gsof/ins_solution_49",
        help="input topic for applanix GSOF49 INS solution",
    )
    return p


def main() -> int:
    if NavigationSolutionGsof49 is None:  # pragma: no cover
        print(
            "ERROR: applanix_msgs not available. Build/install package first:\n"
            "  git clone https://github.com/autowarefoundation/applanix.git\n"
            "  colcon build --packages-select applanix_msgs\n"
            "  source install/setup.bash",
            file=sys.stderr,
        )
        return 2
    args = build_parser().parse_args()
    rclpy.init()
    node = Gsof49ToImu(
        OutputConfig(
            input_topic=args.input_topic,
            output_topic=args.output_topic,
            output_frame_id=args.output_frame_id,
            qos_depth=args.qos_depth,
            output_mode=args.output_mode,
        )
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
