#!/usr/bin/env python3
"""Convert Applanix GSOF49 (NavigationSolutionGsof49) to PoseStamped in a local ENU frame.

This is useful for open datasets that include Applanix INS outputs (lat/lon/alt), e.g.
Autoware Istanbul bags:
  /lvx_client/gsof/ins_solution_49 (applanix_msgs/msg/NavigationSolutionGsof49)

The output pose is expressed in a simple local tangent-like ENU frame whose origin is
the first received sample.

Note:
- This script requires applanix_msgs to be available in the ROS2 environment.
  If you do not have it, build it from source:
    git clone https://github.com/autowarefoundation/applanix.git
    colcon build --packages-select applanix_msgs
    source install/setup.bash
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix

try:
    from applanix_msgs.msg import NavigationSolutionGsof49
except ImportError:  # pragma: no cover
    NavigationSolutionGsof49 = None  # type: ignore[assignment]


EARTH_RADIUS_M = 6378137.0


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion from roll/pitch/yaw using ROS standard (Z-Y-X / yaw-pitch-roll)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = cy * sp * cr + sy * cp * sr
    qz = sy * cp * cr - cy * sp * sr

    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not (n > 0.0) or not math.isfinite(n):
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not (n > 0.0) or not math.isfinite(n):
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def rot_to_quat(R: List[List[float]]) -> Tuple[float, float, float, float]:
    r00, r01, r02 = R[0]
    r10, r11, r12 = R[1]
    r20, r21, r22 = R[2]

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

    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not (n > 0.0) or not math.isfinite(n):
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def matmul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
    return out


@dataclass
class Origin:
    lat_rad: float
    lon_rad: float
    alt_m: float


class ApplanixNavSolutionToPose(Node):
    def __init__(
        self,
        input_topic: str,
        output_topic: str,
        output_frame_id: str,
        qos_depth: int,
        orientation_mode: str,
        origin_navsatfix_topic: str | None,
        origin_navsatfix_qos_depth: int,
    ) -> None:
        super().__init__("applanix_nav_solution_to_pose")
        self._origin: Origin | None = None
        self._use_navsatfix_origin = bool(origin_navsatfix_topic)
        qos = QoSProfile(depth=qos_depth)
        self._pub = self.create_publisher(PoseStamped, output_topic, qos)
        self._sub = self.create_subscription(
            NavigationSolutionGsof49, input_topic, self._callback, qos
        )
        self._origin_sub = None
        if origin_navsatfix_topic:
            qos_origin = QoSProfile(depth=origin_navsatfix_qos_depth)
            self._origin_sub = self.create_subscription(
                NavSatFix,
                origin_navsatfix_topic,
                self._origin_navsatfix_callback,
                qos_origin,
            )
        self._output_frame_id = output_frame_id
        self._orientation_mode = orientation_mode

        origin_source = "ins_solution_49"
        if origin_navsatfix_topic:
            origin_source = f"navsatfix:{origin_navsatfix_topic}"
        self.get_logger().info(
            f"convert NavigationSolutionGsof49 '{input_topic}' -> PoseStamped '{output_topic}' "
            f"frame={output_frame_id} orientation_mode={orientation_mode} origin_source={origin_source}"
        )

    def _origin_navsatfix_callback(self, msg: NavSatFix) -> None:
        if self._origin is not None:
            return
        if not (
            math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)
        ):
            return
        lat_rad = math.radians(float(msg.latitude))
        lon_rad = math.radians(float(msg.longitude))
        alt_m = float(msg.altitude)
        self._origin = Origin(lat_rad=lat_rad, lon_rad=lon_rad, alt_m=alt_m)
        self.get_logger().info(
            f"origin set from NavSatFix lat={msg.latitude:.8f} lon={msg.longitude:.8f} alt={alt_m:.3f}"
        )

    def _callback(self, msg: NavigationSolutionGsof49) -> None:
        lat_deg = float(msg.lla.latitude)
        lon_deg = float(msg.lla.longitude)
        alt_m = float(msg.lla.altitude)
        if not (math.isfinite(lat_deg) and math.isfinite(lon_deg) and math.isfinite(alt_m)):
            return

        if self._origin is None and self._use_navsatfix_origin:
            # Wait for origin from NavSatFix to keep the same ENU origin as GNSS-based estimates.
            return

        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)

        if self._origin is None:
            self._origin = Origin(lat_rad=lat_rad, lon_rad=lon_rad, alt_m=alt_m)
            self.get_logger().info(
                f"origin set lat={lat_deg:.8f} lon={lon_deg:.8f} alt={alt_m:.3f}"
            )

        origin = self._origin
        d_lat = lat_rad - origin.lat_rad
        d_lon = lon_rad - origin.lon_rad

        x_east = EARTH_RADIUS_M * math.cos(origin.lat_rad) * d_lon
        y_north = EARTH_RADIUS_M * d_lat
        z_up = alt_m - origin.alt_m

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self._output_frame_id
        pose.pose.position.x = x_east
        pose.pose.position.y = y_north
        pose.pose.position.z = z_up
        if self._orientation_mode == "identity":
            pose.pose.orientation.w = 1.0
        else:
            roll_rad = math.radians(float(msg.roll))
            pitch_rad = math.radians(float(msg.pitch))
            yaw_rad = math.radians(float(msg.heading))

            qx, qy, qz, qw = quat_from_rpy(roll_rad, pitch_rad, yaw_rad)

            if self._orientation_mode == "ros":
                # Match the applanix_driver_ros "enable_ned2enu_transform" behavior:
                # - Convert navigation frame NED <-> ENU.
                # - Convert body frame from Applanix (FRD) to ROS (FLU).
                enu2ned = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
                applanix2ros = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
                R = quat_to_rot(qx, qy, qz, qw)
                R_corr = matmul(matmul(enu2ned, R), applanix2ros)
                qx, qy, qz, qw = rot_to_quat(R_corr)

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        self._pub.publish(pose)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input-topic", default="/lvx_client/gsof/ins_solution_49")
    p.add_argument("--output-topic", default="/ins_pose")
    p.add_argument("--output-frame-id", default="map")
    p.add_argument("--qos-depth", type=int, default=10)
    p.add_argument(
        "--origin-navsatfix-topic",
        default=None,
        help=(
            "optional NavSatFix topic used to set a shared ENU origin (lat/lon/alt). "
            "Useful when evaluating GNSS-based estimates against INS ground truth."
        ),
    )
    p.add_argument("--origin-navsatfix-qos-depth", type=int, default=10)
    p.add_argument(
        "--orientation-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help=(
            "orientation output mode. "
            "'ros' converts roll/pitch/heading into a ROS-friendly quaternion (ENU + FLU). "
            "'raw_rpy' uses roll/pitch/heading directly without frame conversion. "
            "'identity' publishes an identity quaternion."
        ),
    )
    return p


def main() -> int:
    if NavigationSolutionGsof49 is None:  # pragma: no cover
        print(
            "ERROR: applanix_msgs is not available. Build and source applanix_msgs first.\n"
            "  Example:\n"
            "    git clone https://github.com/autowarefoundation/applanix.git\n"
            "    colcon build --packages-select applanix_msgs\n"
            "    source install/setup.bash\n",
            file=sys.stderr,
        )
        return 2
    args = build_parser().parse_args()
    rclpy.init()
    node = ApplanixNavSolutionToPose(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        output_frame_id=args.output_frame_id,
        qos_depth=args.qos_depth,
        orientation_mode=args.orientation_mode,
        origin_navsatfix_topic=args.origin_navsatfix_topic,
        origin_navsatfix_qos_depth=args.origin_navsatfix_qos_depth,
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
