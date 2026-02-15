#!/usr/bin/env python3
"""Record PoseStamped or Odometry topic into CSV."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile


class PoseCsvRecorder(Node):
    def __init__(
        self,
        topic_name: str,
        msg_type: str,
        output_path: Path,
        qos_depth: int,
        duration_sec: Optional[float],
        max_samples: Optional[int],
    ) -> None:
        super().__init__("pose_csv_recorder")
        self._topic_name = topic_name
        self._msg_type = msg_type
        self._output_path = output_path
        self._duration_sec = duration_sec
        self._max_samples = max_samples
        self._sample_count = 0
        self._start_time = time.monotonic()
        self.done = False

        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self._output_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow(["t_sec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self._f.flush()

        qos = QoSProfile(depth=qos_depth)
        if msg_type == "pose_stamped":
            self._sub = self.create_subscription(PoseStamped, topic_name, self._pose_callback, qos)
        elif msg_type == "odometry":
            self._sub = self.create_subscription(Odometry, topic_name, self._odom_callback, qos)
        else:
            raise ValueError(f"unsupported msg_type: {msg_type}")

        self.get_logger().info(
            f"recording {msg_type} topic '{topic_name}' to {self._output_path}"
        )

    def close(self) -> None:
        self._f.flush()
        self._f.close()
        self.get_logger().info(f"saved {self._sample_count} samples: {self._output_path}")

    def spin_check(self) -> None:
        if self.done:
            return
        if self._duration_sec is not None and self._duration_sec > 0.0:
            if (time.monotonic() - self._start_time) >= self._duration_sec:
                self.done = True
        if self._max_samples is not None and self._sample_count >= self._max_samples:
            self.done = True

    def _write_pose(
        self,
        sec: int,
        nanosec: int,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> None:
        if self.done:
            return
        t_sec = float(sec) + float(nanosec) * 1e-9
        self._writer.writerow(
            [
                f"{t_sec:.9f}",
                f"{x:.9f}",
                f"{y:.9f}",
                f"{z:.9f}",
                f"{qx:.9f}",
                f"{qy:.9f}",
                f"{qz:.9f}",
                f"{qw:.9f}",
            ]
        )
        self._sample_count += 1
        if self._sample_count % 100 == 0:
            self._f.flush()

    def _pose_callback(self, msg: PoseStamped) -> None:
        self._write_pose(
            sec=msg.header.stamp.sec,
            nanosec=msg.header.stamp.nanosec,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            qx=msg.pose.orientation.x,
            qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._write_pose(
            sec=msg.header.stamp.sec,
            nanosec=msg.header.stamp.nanosec,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            qx=pose.orientation.x,
            qy=pose.orientation.y,
            qz=pose.orientation.z,
            qw=pose.orientation.w,
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--topic", required=True)
    p.add_argument("--msg-type", choices=["pose_stamped", "odometry"], default="pose_stamped")
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--qos-depth", type=int, default=10)
    p.add_argument("--duration-sec", type=float, default=None)
    p.add_argument("--max-samples", type=int, default=None)
    return p


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()

    node = PoseCsvRecorder(
        topic_name=args.topic,
        msg_type=args.msg_type,
        output_path=args.output,
        qos_depth=args.qos_depth,
        duration_sec=args.duration_sec,
        max_samples=args.max_samples,
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.spin_check()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
