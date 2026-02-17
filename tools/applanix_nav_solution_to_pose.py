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

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from applanix_msgs.msg import NavigationSolutionGsof49
except ImportError:  # pragma: no cover
    NavigationSolutionGsof49 = None  # type: ignore[assignment]


EARTH_RADIUS_M = 6378137.0


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
    ) -> None:
        super().__init__("applanix_nav_solution_to_pose")
        self._origin: Origin | None = None
        qos = QoSProfile(depth=qos_depth)
        self._pub = self.create_publisher(PoseStamped, output_topic, qos)
        self._sub = self.create_subscription(
            NavigationSolutionGsof49, input_topic, self._callback, qos
        )
        self._output_frame_id = output_frame_id
        self.get_logger().info(
            f"convert NavigationSolutionGsof49 '{input_topic}' -> PoseStamped '{output_topic}' frame={output_frame_id}"
        )

    def _callback(self, msg: NavigationSolutionGsof49) -> None:
        lat_deg = float(msg.lla.latitude)
        lon_deg = float(msg.lla.longitude)
        alt_m = float(msg.lla.altitude)
        if not (math.isfinite(lat_deg) and math.isfinite(lon_deg) and math.isfinite(alt_m)):
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
        pose.pose.orientation.w = 1.0
        self._pub.publish(pose)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input-topic", default="/lvx_client/gsof/ins_solution_49")
    p.add_argument("--output-topic", default="/ins_pose")
    p.add_argument("--output-frame-id", default="map")
    p.add_argument("--qos-depth", type=int, default=10)
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
