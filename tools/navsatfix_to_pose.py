#!/usr/bin/env python3
"""Convert NavSatFix to PoseStamped in a local tangent-like frame."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS_M = 6378137.0


@dataclass
class Origin:
    lat_rad: float
    lon_rad: float
    alt_m: float


class NavSatFixToPose(Node):
    def __init__(
        self,
        input_topic: str,
        output_topic: str,
        output_frame_id: str,
        qos_depth: int,
    ) -> None:
        super().__init__("navsatfix_to_pose")
        self._origin: Origin | None = None
        qos = QoSProfile(depth=qos_depth)
        self._pub = self.create_publisher(PoseStamped, output_topic, qos)
        self._sub = self.create_subscription(NavSatFix, input_topic, self._callback, qos)
        self._output_frame_id = output_frame_id
        self.get_logger().info(
            f"convert NavSatFix '{input_topic}' -> PoseStamped '{output_topic}' frame={output_frame_id}"
        )

    def _callback(self, msg: NavSatFix) -> None:
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and math.isfinite(msg.altitude)):
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt_m = float(msg.altitude)

        if self._origin is None:
            self._origin = Origin(lat_rad=lat_rad, lon_rad=lon_rad, alt_m=alt_m)
            self.get_logger().info(
                f"origin set lat={msg.latitude:.8f} lon={msg.longitude:.8f} alt={alt_m:.3f}"
            )

        origin = self._origin
        d_lat = lat_rad - origin.lat_rad
        d_lon = lon_rad - origin.lon_rad

        x_east = EARTH_RADIUS_M * math.cos(origin.lat_rad) * d_lon
        y_north = EARTH_RADIUS_M * d_lat
        z_up = alt_m - origin.alt_m

        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self._output_frame_id
        else:
            pose.header.frame_id = self._output_frame_id
        pose.pose.position.x = x_east
        pose.pose.position.y = y_north
        pose.pose.position.z = z_up
        pose.pose.orientation.w = 1.0
        self._pub.publish(pose)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input-topic", default="/fix")
    p.add_argument("--output-topic", default="/gnss_pose")
    p.add_argument("--output-frame-id", default="map")
    p.add_argument("--qos-depth", type=int, default=10)
    return p


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = NavSatFixToPose(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        output_frame_id=args.output_frame_id,
        qos_depth=args.qos_depth,
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
