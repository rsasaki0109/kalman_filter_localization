#!/usr/bin/env python3
"""Run EKF parameter sweep on a ROS2 bag and evaluate trajectory error metrics."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import html
import itertools
import json
import os
import math
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
RECORD_SCRIPT = SCRIPT_DIR / "record_pose_csv.py"
EVAL_SCRIPT = SCRIPT_DIR / "evaluate_trajectory.py"
NAVSATFIX_SCRIPT = SCRIPT_DIR / "navsatfix_to_pose.py"
PLOT_SCRIPT = SCRIPT_DIR / "plot_pose_csv.py"
APPLANIX_SCRIPT = SCRIPT_DIR / "applanix_nav_solution_to_pose.py"
GSOF49_TO_IMU_SCRIPT = SCRIPT_DIR / "gsof49_to_imu.py"
REPORT_NAME_PREFIX = "open_data_report"


@dataclass
class ManagedProcess:
    name: str
    proc: subprocess.Popen
    log_fp: object


def parse_initial_pose(text: str) -> Tuple[float, float, float, float, float, float, float]:
    items = [v.strip() for v in text.split(",")]
    if len(items) != 7:
        raise ValueError(
            "--initial-pose must contain exactly 7 comma-separated values: x,y,z,qx,qy,qz,qw"
        )
    values = tuple(float(v) for v in items)
    return values  # type: ignore[return-value]


def parse_static_tf_args(text: str) -> List[str]:
    items = [v.strip() for v in text.split(",")]
    if len(items) != 9:
        raise ValueError(
            "--static-tf must contain 9 comma-separated values: "
            "x,y,z,qx,qy,qz,qw,frame_id,child_frame_id"
        )
    return items


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if sinp <= -1.0:
        pitch = -math.pi / 2.0
    elif sinp >= 1.0:
        pitch = math.pi / 2.0
    else:
        pitch = math.asin(sinp)

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return roll, pitch, yaw


def read_initial_yaw_from_pose_topic(
    *, topic: str, msg_type: str, timeout_sec: float, qos_depth: int
) -> float:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class InitialYawListener(Node):
        def __init__(self) -> None:
            super().__init__("initial_yaw_listener")
            self._yaw: Optional[float] = None
            qos = int(qos_depth) if qos_depth > 0 else 10
            if msg_type == "pose_stamped":
                self._sub = self.create_subscription(
                    PoseStamped, topic, self._on_pose_stamped, qos
                )
            elif msg_type == "odometry":
                self._sub = self.create_subscription(Odometry, topic, self._on_odometry, qos)
            elif msg_type == "imu":
                self._sub = self.create_subscription(Imu, topic, self._on_imu, qos)
            else:  # pragma: no cover
                raise ValueError(f"unsupported msg type for initial yaw source: {msg_type}")

        def _on_pose_stamped(self, msg: PoseStamped) -> None:
            self._on_quat(msg.pose.orientation)

        def _on_odometry(self, msg: Odometry) -> None:
            self._on_quat(msg.pose.pose.orientation)

        def _on_imu(self, msg: Imu) -> None:
            self._on_quat(msg.orientation)

        def _on_quat(self, q) -> None:
            if self._yaw is None:
                if not (
                    math.isfinite(q.x)
                    and math.isfinite(q.y)
                    and math.isfinite(q.z)
                    and math.isfinite(q.w)
                ):
                    return
                if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) <= 0.0:
                    return
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._yaw = math.atan2(siny_cosp, cosy_cosp)

    rclpy.init()
    node = InitialYawListener()
    deadline = time.time() + max(0.0, float(timeout_sec))
    try:
        while rclpy.ok() and time.time() < deadline and node._yaw is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node._yaw is None:
        raise TimeoutError(f"timed out waiting for initial yaw from {topic} ({msg_type})")
    return float(node._yaw)


def ros_value(v: object) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        text = f"{v:.12g}"
        if "e" not in text and "." not in text:
            text += ".0"
        return text
    return str(v)


def load_param_grid(grid_path: Path) -> Tuple[List[str], List[Tuple[object, ...]]]:
    raw = json.loads(grid_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{grid_path} must contain a JSON object")
    if not raw:
        raise ValueError(f"{grid_path} is empty")

    keys = list(raw.keys())
    value_lists: List[List[object]] = []
    for key in keys:
        values = raw[key]
        if not isinstance(values, list) or not values:
            raise ValueError(f"grid key '{key}' must map to a non-empty array")
        value_lists.append(values)
    combinations = list(itertools.product(*value_lists))
    return keys, combinations


def start_background_process(name: str, cmd: Sequence[str], log_path: Path) -> ManagedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fp = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(  # pylint: disable=consider-using-with
        list(cmd),
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return ManagedProcess(name=name, proc=proc, log_fp=log_fp)


def stop_background_process(p: ManagedProcess, timeout_sec: float = 5.0) -> None:
    try:
        if p.proc.poll() is None:
            os.killpg(p.proc.pid, signal.SIGINT)
            p.proc.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(p.proc.pid, signal.SIGTERM)
            p.proc.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            os.killpg(p.proc.pid, signal.SIGKILL)
            p.proc.wait(timeout=timeout_sec)
    finally:
        p.log_fp.flush()
        p.log_fp.close()


def write_summary_csv(path: Path, rows: List[Dict[str, object]], fieldnames: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_html_report(
    *,
    output_dir: Path,
    stamp: str,
    bag_path: Path,
    param_grid_json: Path,
    summary_csv: Path,
    ranking_csv: Path,
    ranking_nobias_csv: Path,
    best: Optional[Dict[str, object]],
    best_nobias: Optional[Dict[str, object]],
    keys: Sequence[str],
    successful_sorted: Sequence[Dict[str, object]],
    successful_nobias_sorted: Sequence[Dict[str, object]],
) -> Path:
    report_path = output_dir / f"{REPORT_NAME_PREFIX}_{stamp}.html"

    def esc(s: object) -> str:
        return html.escape("" if s is None else str(s))

    def rel(p: Path) -> str:
        try:
            return str(p.relative_to(output_dir))
        except Exception:  # pylint: disable=broad-except
            return str(p)

    def img_section(run_id: str) -> str:
        run_dir = output_dir / run_id
        xy = run_dir / f"{run_id}_trajectory_xy.png"
        ts = run_dir / f"{run_id}_timeseries_z_rpy.png"

        def img_or_missing(p: Path, alt: str) -> str:
            if p.exists():
                return f'<img src="{esc(rel(p))}" alt="{esc(alt)}" loading="lazy" />'
            return '<div class="missing">(missing)</div>'

        return "\n".join(
            [
                '<div class="imgs">',
                img_or_missing(xy, f"{run_id} trajectory XY"),
                img_or_missing(ts, f"{run_id} timeseries z+RPY"),
                "</div>",
            ]
        )

    def kv_run(row: Dict[str, object], *, show_images: bool = True) -> str:
        run_id = str(row.get("run_id", ""))
        parts = [
            f"<div><b>run_id</b>: <code>{esc(run_id)}</code></div>",
            f"<div><b>rmse_3d_m</b>: <code>{esc(row.get('rmse_3d_m',''))}</code></div>",
            f"<div><b>rmse_3d_nobias_m</b>: <code>{esc(row.get('rmse_3d_nobias_m',''))}</code></div>",
            f"<div><b>matched</b>: <code>{esc(row.get('matched_samples',''))}</code></div>",
            f"<div><b>yaw_ref</b>: <code>{esc(row.get('yaw_reference',''))}</code></div>",
            f"<div><b>yaw_rmse_deg</b>: <code>{esc(row.get('yaw_rmse_deg',''))}</code></div>",
            f"<div><b>att_angle_rmse_deg</b>: <code>{esc(row.get('attitude_angle_rmse_deg',''))}</code></div>",
            f"<div><b>initial_yaw</b>: <code>{esc(row.get('initial_yaw_label', ''))}</code></div>",
            f"<div><b>initial_yaw_deg</b>: <code>{esc(row.get('initial_yaw_deg', ''))}</code></div>",
        ]
        params = {k: row.get(k, "") for k in keys}
        parts.append(f"<div><b>params</b>: <code>{esc(params)}</code></div>")
        if show_images and run_id:
            parts.append(img_section(run_id))
        return "\n".join(parts)

    def kv_best_pair(
        best_row: Optional[Dict[str, object]],
        best_nobias_row: Optional[Dict[str, object]],
    ) -> str:
        if best_row is None and best_nobias_row is None:
            return "<div class=\"missing\">(no successful runs)</div>"

        blocks: List[str] = []
        if best_row is not None:
            blocks.append("<div class=\"subpanel\"><h3>Best by rmse_3d_m</h3>")
            blocks.append(kv_run(best_row, show_images=True))
            blocks.append("</div>")

        if best_nobias_row is None:
            return "\n".join(blocks)

        best_id = str(best_row.get("run_id", "")) if best_row is not None else ""
        nobias_id = str(best_nobias_row.get("run_id", ""))
        show_nobias_image = not best_id or (nobias_id == best_id)
        if best_id == nobias_id:
            return "\n".join(blocks)

        blocks.append("<div class=\"subpanel\"><h3>Best by rmse_3d_nobias_m</h3>")
        blocks.append(
            kv_run(
                best_nobias_row,
                show_images=show_nobias_image,
            )
        )
        blocks.append("</div>")
        return "\n".join(blocks)

    def top_table(rows: Sequence[Dict[str, object]], title: str) -> str:
        head = (
            "<tr>"
            "<th>rank</th>"
            "<th>run_id</th>"
            "<th>rmse_3d_m</th>"
            "<th>rmse_3d_nobias_m</th>"
            "<th>matched</th>"
            "<th>yaw_rmse_deg</th>"
            "</tr>"
        )
        body_rows: List[str] = []
        for i, r in enumerate(rows[:10], start=1):
            body_rows.append(
                "<tr>"
                f"<td>{i}</td>"
                f"<td><code>{esc(r.get('run_id',''))}</code></td>"
                f"<td><code>{esc(r.get('rmse_3d_m',''))}</code></td>"
                f"<td><code>{esc(r.get('rmse_3d_nobias_m',''))}</code></td>"
                f"<td><code>{esc(r.get('matched_samples',''))}</code></td>"
                f"<td><code>{esc(r.get('yaw_rmse_deg',''))}</code></td>"
                "</tr>"
            )
        body = "\n".join(body_rows) if body_rows else '<tr><td colspan="6">(empty)</td></tr>'
        return "\n".join(
            [
                f"<h3>{esc(title)}</h3>",
                '<table class="top">',
                "<thead>",
                head,
                "</thead>",
                "<tbody>",
                body,
                "</tbody>",
                "</table>",
            ]
        )

    css = """
    :root { color-scheme: light; }
    body { font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, Cantarell, Noto Sans, sans-serif; margin: 24px; }
    h1 { margin: 0 0 8px 0; }
    .sub { color: #333; margin: 0 0 18px 0; }
    code { background: #f3f3f3; padding: 2px 6px; border-radius: 6px; }
    .grid { display: grid; grid-template-columns: 1fr; gap: 16px; }
    .subpanel { border: 1px dashed #ddd; border-radius: 10px; padding: 10px; margin-top: 10px; }
    .panel { border: 1px solid #ddd; border-radius: 12px; padding: 14px; background: #fff; }
    .imgs { display: grid; grid-template-columns: 1fr; gap: 12px; margin-top: 10px; }
    img { width: 100%; height: auto; border: 1px solid #eee; border-radius: 10px; }
    .missing { color: #888; font-style: italic; padding: 12px; border: 1px dashed #ccc; border-radius: 10px; }
    table.top { border-collapse: collapse; width: 100%; }
    table.top th, table.top td { border: 1px solid #ddd; padding: 6px 8px; text-align: left; }
    table.top th { background: #fafafa; }
    a { color: #0645ad; }
    """

    summary_rel = esc(rel(summary_csv))
    ranking_rel = esc(rel(ranking_csv))
    ranking_nb_rel = esc(rel(ranking_nobias_csv))

    best_html = kv_best_pair(best, best_nobias)

    doc = "\n".join(
        [
            "<!doctype html>",
            "<html>",
            "<head>",
            "<meta charset=\"utf-8\" />",
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />",
            f"<title>{REPORT_NAME_PREFIX}_{esc(stamp)}</title>",
            f"<style>{css}</style>",
            "</head>",
            "<body>",
            f"<h1>Open-Data Report ({esc(stamp)})</h1>",
            "<p class=\"sub\">"
            f"bag_path: <code>{esc(bag_path)}</code><br/>"
            f"param_grid_json: <code>{esc(param_grid_json)}</code><br/>"
            f"summary_csv: <a href=\"{summary_rel}\"><code>{summary_rel}</code></a><br/>"
            f"ranking_csv: <a href=\"{ranking_rel}\"><code>{ranking_rel}</code></a><br/>"
            f"ranking_nobias_csv: <a href=\"{ranking_nb_rel}\"><code>{ranking_nb_rel}</code></a>"
            "</p>",
            "<p class=\"sub\">"
            "<b>Best run selection:</b> first is best by <code>rmse_3d_m</code> "
            "(bias kept), second is best by <code>rmse_3d_nobias_m</code> "
            "(bias removed before RMSE)."
            "</p>",
            "<div class=\"grid\">",
            "<div class=\"panel\">",
            "<h2>Best Runs</h2>",
            best_html,
            "</div>",
            "</div>",
            top_table(successful_sorted, "Top 10 by rmse_3d_m"),
            top_table(successful_nobias_sorted, "Top 10 by rmse_3d_nobias_m"),
            "</body>",
            "</html>",
        ]
    )
    report_path.write_text(doc, encoding="utf-8")
    return report_path


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag-path", required=True, type=Path)
    p.add_argument("--param-grid-json", required=True, type=Path)
    p.add_argument("--output-dir", required=True, type=Path)

    p.add_argument("--estimated-topic", default="/ekf_localization/current_pose")
    p.add_argument("--estimated-qos-depth", type=int, default=10)
    p.add_argument("--ground-truth-topic", default="/gnss_pose")
    p.add_argument("--ground-truth-msg-type", choices=["pose_stamped", "odometry"], default="pose_stamped")
    p.add_argument("--ground-truth-qos-depth", type=int, default=10)
    p.add_argument(
        "--attitude-reference-topic",
        default=None,
        help="optional topic to record as attitude reference CSV for plotting (e.g. IMU orientation topic)",
    )
    p.add_argument(
        "--attitude-reference-msg-type",
        choices=["pose_stamped", "odometry", "imu"],
        default="imu",
        help="message type for --attitude-reference-topic (default: imu)",
    )
    p.add_argument("--attitude-reference-qos-depth", type=int, default=10)

    p.add_argument("--initial-pose-topic", default="/ekf_localization/initial_pose")
    p.add_argument("--initial-pose", default="0,0,0,0,0,0,1")
    p.add_argument("--publish-initial-pose", action="store_true", default=True)
    p.add_argument("--no-publish-initial-pose", action="store_false", dest="publish_initial_pose")
    p.add_argument(
        "--initial-yaw-source-topic",
        default=None,
        help=(
            "optional topic to derive initial yaw from (e.g. /ins_pose). "
            "If omitted and ground truth is /ins_pose, defaults to /ins_pose."
        ),
    )
    p.add_argument(
        "--initial-yaw-source-msg-type",
        choices=["pose_stamped", "odometry", "imu"],
        default="pose_stamped",
        help="message type for --initial-yaw-source-topic (default: pose_stamped)",
    )
    p.add_argument("--initial-yaw-qos-depth", type=int, default=10)
    p.add_argument(
        "--initial-yaw-timeout-sec",
        type=float,
        default=30.0,
        help="timeout for initial yaw sample read in seconds (default: 30.0)",
    )
    p.add_argument(
        "--initial-pose-wait-subscriptions",
        type=int,
        default=1,
        help="ros2 topic pub wait_maching_subscriptions value for initial pose (default: 1)",
    )

    p.add_argument("--reference-frame-id", default="map")
    p.add_argument("--robot-frame-id", default="base_link")
    p.add_argument("--imu-topic", default="/imu")
    p.add_argument("--gnss-topic", default="/gnss_pose")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--pub-period", type=int, default=10)
    p.add_argument("--use-gnss", action="store_true", default=True)
    p.add_argument("--no-use-gnss", action="store_false", dest="use_gnss")
    p.add_argument("--use-odom", action="store_true", default=False)
    p.add_argument("--no-use-odom", action="store_false", dest="use_odom")
    p.add_argument(
        "--ekf-output-stamp-source",
        choices=["latest_input", "imu", "ros_time"],
        default="latest_input",
        help=(
            "timestamp source for EKF current_pose output. "
            "latest_input matches the latest received message stamp (default). "
            "imu uses the latest IMU stamp only. "
            "ros_time uses the node's ROS time (use_sim_time) clock."
        ),
    )

    p.add_argument("--enable-static-tf", action="store_true", default=True)
    p.add_argument("--disable-static-tf", action="store_false", dest="enable_static_tf")
    p.add_argument(
        "--static-tf",
        default="0,0,0,0,0,0,1,base_link,imu_link",
        help="x,y,z,qx,qy,qz,qw,frame_id,child_frame_id",
    )

    p.add_argument(
        "--enable-navsatfix-to-pose",
        action="store_true",
        default=False,
        help="start tools/navsatfix_to_pose.py during each run (to convert NavSatFix -> PoseStamped)",
    )
    p.add_argument("--navsatfix-input-topic", default="/fix")
    p.add_argument("--navsatfix-output-topic", default="/gnss_pose")
    p.add_argument("--navsatfix-qos-depth", type=int, default=10)

    p.add_argument(
        "--enable-applanix-to-pose",
        action="store_true",
        default=False,
        help="start tools/applanix_nav_solution_to_pose.py during each run (to convert Applanix GSOF49 -> PoseStamped)",
    )
    p.add_argument("--applanix-input-topic", default="/lvx_client/gsof/ins_solution_49")
    p.add_argument("--applanix-output-topic", default="/ins_pose")
    p.add_argument("--applanix-qos-depth", type=int, default=10)
    p.add_argument(
        "--applanix-orientation-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help="forwarded to tools/applanix_nav_solution_to_pose.py --orientation-mode",
    )
    p.add_argument(
        "--applanix-origin-navsatfix-topic",
        default=None,
        help=(
            "optional NavSatFix topic used by tools/applanix_nav_solution_to_pose.py to set a shared ENU origin. "
            "Useful for INS ground truth evaluation to reduce frame offset vs GNSS-origin estimates."
        ),
    )
    p.add_argument("--applanix-origin-navsatfix-qos-depth", type=int, default=10)
    p.add_argument(
        "--enable-applanix-to-imu",
        action="store_true",
        default=False,
        help=(
            "start tools/gsof49_to_imu.py during each run (to convert GSOF49 INS -> Imu for EKF input)"
        ),
    )
    p.add_argument(
        "--applanix-imu-input-topic",
        default="/lvx_client/gsof/ins_solution_49",
        help="input topic for gsof49_to_imu.py",
    )
    p.add_argument(
        "--applanix-imu-output-topic",
        default="/ins_imu",
        help="output topic for gsof49_to_imu.py",
    )
    p.add_argument(
        "--applanix-imu-output-frame-id",
        default="base_link",
        help="output_frame_id for gsof49_to_imu.py",
    )
    p.add_argument("--applanix-imu-qos-depth", type=int, default=10)
    p.add_argument(
        "--applanix-imu-output-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help="output-mode for gsof49_to_imu.py",
    )

    p.add_argument("--play-rate", type=float, default=1.0)
    p.add_argument(
        "--play-topics",
        nargs="+",
        default=None,
        help="optional list of topics to replay (reduces playback load)",
    )
    p.add_argument("--bag-start-offset-sec", type=float, default=0.0)
    p.add_argument("--startup-sec", type=float, default=2.0)
    p.add_argument("--tail-sec", type=float, default=1.0)
    p.add_argument("--eval-max-time-gap-sec", type=float, default=0.1)
    p.add_argument(
        "--attitude-min-speed-mps",
        type=float,
        default=0.0,
        help=(
            "when > 0, compute/plot attitude errors only when ground-truth horizontal speed >= this threshold. "
            "Useful since yaw can be unobservable at standstill."
        ),
    )
    p.add_argument("--max-runs", type=int, default=0, help="0 means all combinations")
    p.add_argument("--run-prefix", default="run")
    p.add_argument(
        "--plot-best",
        action="store_true",
        default=False,
        help=(
            "generate plots for the best run(s) (requires tools/plot_pose_csv.py + matplotlib). "
            "When available, plots both: best by rmse_3d_m and best by rmse_3d_nobias_m."
        ),
    )
    return p


def main() -> int:
    args = build_parser().parse_args()

    if not args.bag_path.exists():
        print(f"ERROR: bag not found: {args.bag_path}", file=sys.stderr)
        return 2
    if not args.param_grid_json.exists():
        print(f"ERROR: param grid not found: {args.param_grid_json}", file=sys.stderr)
        return 2
    if not RECORD_SCRIPT.exists() or not EVAL_SCRIPT.exists():
        print("ERROR: helper scripts are missing in tools/", file=sys.stderr)
        return 2
    if args.enable_navsatfix_to_pose and not NAVSATFIX_SCRIPT.exists():
        print("ERROR: tools/navsatfix_to_pose.py is missing", file=sys.stderr)
        return 2
    if args.enable_applanix_to_pose and not APPLANIX_SCRIPT.exists():
        print("ERROR: tools/applanix_nav_solution_to_pose.py is missing", file=sys.stderr)
        return 2
    if args.enable_applanix_to_imu and not GSOF49_TO_IMU_SCRIPT.exists():
        print("ERROR: tools/gsof49_to_imu.py is missing", file=sys.stderr)
        return 2
    if args.play_rate <= 0.0:
        print("ERROR: --play-rate must be > 0", file=sys.stderr)
        return 2
    if args.bag_start_offset_sec < 0.0:
        print("ERROR: --bag-start-offset-sec must be >= 0", file=sys.stderr)
        return 2

    try:
        initial_pose = parse_initial_pose(args.initial_pose)
        static_tf_args = parse_static_tf_args(args.static_tf)
        keys, combinations = load_param_grid(args.param_grid_json)
    except Exception as e:  # pylint: disable=broad-except
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    if args.max_runs > 0:
        combinations = combinations[: args.max_runs]
    if args.initial_yaw_source_topic is None and args.ground_truth_topic == "/ins_pose":
        args.initial_yaw_source_topic = "/ins_pose"

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "summary.csv"
    ranking_path = args.output_dir / "ranking_by_rmse_3d.csv"
    ranking_nobias_path = args.output_dir / "ranking_by_rmse_3d_nobias.csv"

    print(f"bag_path: {args.bag_path}")
    print(f"param_grid_json: {args.param_grid_json}")
    print(f"total_runs: {len(combinations)}")
    print(f"output_dir: {args.output_dir}")

    rows: List[Dict[str, object]] = []
    cached_initial_yaw: Optional[float] = None
    cached_initial_yaw_label = ""
    cached_initial_yaw_deg = ""
    fixed_ros_params = {
        "reference_frame_id": args.reference_frame_id,
        "robot_frame_id": args.robot_frame_id,
        "imu_topic": args.imu_topic,
        "gnss_pose_topic": args.gnss_topic,
        "odom_topic": args.odom_topic,
        "pub_period": args.pub_period,
        "use_gnss": args.use_gnss,
        "use_odom": args.use_odom,
        "output_stamp_source": args.ekf_output_stamp_source,
    }

    for idx, values in enumerate(combinations, start=1):
        run_id = f"{args.run_prefix}_{idx:03d}"
        run_dir = args.output_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=True)

        dynamic_params = {k: v for k, v in zip(keys, values)}
        all_ros_params = dict(fixed_ros_params)
        all_ros_params.update(dynamic_params)

        print(f"\n[{idx}/{len(combinations)}] {run_id} params={dynamic_params}")

        est_csv = run_dir / "estimated.csv"
        gt_csv = run_dir / "ground_truth.csv"
        att_csv = run_dir / "attitude_reference.csv"
        metrics_json = run_dir / "metrics.json"

        processes: List[ManagedProcess] = []
        status = "ok"

        try:
            if args.enable_static_tf:
                tf_cmd = ["ros2", "run", "tf2_ros", "static_transform_publisher"] + static_tf_args
                processes.append(start_background_process("static_tf", tf_cmd, run_dir / "static_tf.log"))

            if args.enable_navsatfix_to_pose:
                conv_cmd = [
                    sys.executable,
                    str(NAVSATFIX_SCRIPT),
                    "--input-topic",
                    args.navsatfix_input_topic,
                    "--output-topic",
                    args.navsatfix_output_topic,
                    "--output-frame-id",
                    args.reference_frame_id,
                    "--qos-depth",
                    str(args.navsatfix_qos_depth),
                ]
                processes.append(
                    start_background_process(
                        "navsatfix_to_pose", conv_cmd, run_dir / "navsatfix_to_pose.log"
                    )
                )

            if args.enable_applanix_to_pose:
                apx_cmd = [
                    sys.executable,
                    str(APPLANIX_SCRIPT),
                    "--input-topic",
                    args.applanix_input_topic,
                    "--output-topic",
                    args.applanix_output_topic,
                    "--output-frame-id",
                    args.reference_frame_id,
                    "--qos-depth",
                    str(args.applanix_qos_depth),
                    "--orientation-mode",
                    str(args.applanix_orientation_mode),
                ]
                if args.applanix_origin_navsatfix_topic:
                    apx_cmd += [
                        "--origin-navsatfix-topic",
                        args.applanix_origin_navsatfix_topic,
                        "--origin-navsatfix-qos-depth",
                        str(args.applanix_origin_navsatfix_qos_depth),
                    ]
                processes.append(
                    start_background_process(
                        "applanix_to_pose", apx_cmd, run_dir / "applanix_to_pose.log"
                    )
                )

            if args.enable_applanix_to_imu:
                imu_cmd = [
                    sys.executable,
                    str(GSOF49_TO_IMU_SCRIPT),
                    "--input-topic",
                    args.applanix_imu_input_topic,
                    "--output-topic",
                    args.applanix_imu_output_topic,
                    "--output-frame-id",
                    args.applanix_imu_output_frame_id,
                    "--qos-depth",
                    str(args.applanix_imu_qos_depth),
                    "--output-mode",
                    str(args.applanix_imu_output_mode),
                ]
                processes.append(
                    start_background_process("gsof49_to_imu", imu_cmd, run_dir / "gsof49_to_imu.log")
                )

            ekf_cmd: List[str] = [
                "ros2",
                "run",
                "kalman_filter_localization",
                "ekf_localization_node",
                "--ros-args",
            ]
            for k, v in all_ros_params.items():
                ekf_cmd.extend(["-p", f"{k}:={ros_value(v)}"])
            processes.append(start_background_process("ekf", ekf_cmd, run_dir / "ekf.log"))

            rec_est_cmd = [
                sys.executable,
                str(RECORD_SCRIPT),
                "--topic",
                args.estimated_topic,
                "--msg-type",
                "pose_stamped",
                "--output",
                str(est_csv),
                "--qos-depth",
                str(args.estimated_qos_depth),
            ]
            processes.append(
                start_background_process("record_est", rec_est_cmd, run_dir / "record_est.log")
            )

            rec_gt_cmd = [
                sys.executable,
                str(RECORD_SCRIPT),
                "--topic",
                args.ground_truth_topic,
                "--msg-type",
                args.ground_truth_msg_type,
                "--output",
                str(gt_csv),
                "--qos-depth",
                str(args.ground_truth_qos_depth),
            ]
            processes.append(
                start_background_process("record_gt", rec_gt_cmd, run_dir / "record_gt.log")
            )

            if args.attitude_reference_topic:
                rec_att_cmd = [
                    sys.executable,
                    str(RECORD_SCRIPT),
                    "--topic",
                    args.attitude_reference_topic,
                    "--msg-type",
                    args.attitude_reference_msg_type,
                    "--output",
                    str(att_csv),
                    "--qos-depth",
                    str(args.attitude_reference_qos_depth),
                ]
                processes.append(
                    start_background_process(
                        "record_attitude_ref",
                        rec_att_cmd,
                        run_dir / "record_attitude_ref.log",
                    )
                )

            time.sleep(args.startup_sec)

            play_cmd = [
                "ros2",
                "bag",
                "play",
                str(args.bag_path),
                "--clock",
                "--rate",
                f"{args.play_rate:.12g}",
            ]
            if args.bag_start_offset_sec > 0.0:
                play_cmd.extend(["--start-offset", f"{args.bag_start_offset_sec:.12g}"])
            if args.play_topics:
                play_cmd.extend(["--topics", *args.play_topics])

            play_proc = start_background_process(
                "bag_play", play_cmd, run_dir / "bag_play.log"
            )
            processes.append(play_proc)

            if cached_initial_yaw is None and args.initial_yaw_source_topic:
                _, _, yaw_from_initial = quat_to_rpy(
                    initial_pose[3], initial_pose[4], initial_pose[5], initial_pose[6]
                )
                try:
                    cached_initial_yaw = read_initial_yaw_from_pose_topic(
                        topic=args.initial_yaw_source_topic,
                        msg_type=args.initial_yaw_source_msg_type,
                        timeout_sec=max(args.initial_yaw_timeout_sec, 1.0),
                        qos_depth=args.initial_yaw_qos_depth,
                    )
                    cached_initial_yaw_label = "yaw_init = yaw_poslv"
                    cached_initial_yaw_deg = f"{math.degrees(cached_initial_yaw):.6f}"
                except Exception as e:  # pylint: disable=broad-except
                    print(
                        f"  warning: could not set initial yaw from topic: {e}"
                    )
                    cached_initial_yaw = None
                    cached_initial_yaw_label = "yaw_init = fallback (pose arg)"
                    cached_initial_yaw_deg = f"{math.degrees(yaw_from_initial):.6f}"

            if args.publish_initial_pose:
                x, y, z, qx, qy, qz, qw = initial_pose
                initial_yaw_label = "yaw_init = from --initial-pose"
                initial_yaw_deg = ""
                if args.initial_yaw_source_topic:
                    if cached_initial_yaw is not None:
                        roll, pitch, _ = quat_to_rpy(qx, qy, qz, qw)
                        qx, qy, qz, qw = quat_from_rpy(roll, pitch, cached_initial_yaw)
                        initial_yaw_label = cached_initial_yaw_label
                        initial_yaw_deg = cached_initial_yaw_deg
                    else:
                        initial_yaw_label = cached_initial_yaw_label
                        initial_yaw_deg = cached_initial_yaw_deg
                if not initial_yaw_deg:
                    _, _, yaw_from_initial = quat_to_rpy(qx, qy, qz, qw)
                    initial_yaw_deg = f"{math.degrees(yaw_from_initial):.6f}"

                (run_dir / "initial_yaw.log").write_text(
                    f"label={initial_yaw_label}\n"
                    f"value_deg={initial_yaw_deg}\n"
                    f"topic={args.initial_yaw_source_topic}\n",
                    encoding="utf-8",
                )

                pose_yaml = (
                    "{header: {frame_id: '"
                    + args.reference_frame_id
                    + "'}, pose: {position: {x: "
                    + f"{x}"
                    + ", y: "
                    + f"{y}"
                    + ", z: "
                    + f"{z}"
                    + "}, orientation: {x: "
                    + f"{qx}"
                    + ", y: "
                    + f"{qy}"
                    + ", z: "
                    + f"{qz}"
                    + ", w: "
                    + f"{qw}"
                    + "}}}"
                )
                pub_cmd = [
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "--wait-matching-subscriptions",
                    str(args.initial_pose_wait_subscriptions),
                    args.initial_pose_topic,
                    "geometry_msgs/msg/PoseStamped",
                    pose_yaml,
                ]
                try:
                    pub_result = subprocess.run(
                        pub_cmd,
                        check=False,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        timeout=10,
                    )
                    (run_dir / "initial_pose_pub.log").write_text(
                        pub_result.stdout, encoding="utf-8"
                    )
                    if pub_result.returncode != 0:
                        print(
                            f"  warning: initial pose publish failed (code={pub_result.returncode})"
                        )
                except subprocess.TimeoutExpired as e:
                    timeout_text = e.stdout or ""
                    (run_dir / "initial_pose_pub.log").write_text(
                        timeout_text + "\nTIMEOUT\n", encoding="utf-8"
                    )
                    print("  warning: initial pose publish timed out")

            play_proc.proc.wait()
            time.sleep(args.tail_sec)

        except subprocess.CalledProcessError as e:
            status = f"failed_command({e.returncode})"
            print(f"  run failed: {status}")
        except Exception as e:  # pylint: disable=broad-except
            status = f"failed({e})"
            print(f"  run failed: {status}")
        finally:
            for p in reversed(processes):
                stop_background_process(p)

        metrics: Dict[str, object] = {
            "initial_yaw_label": "",
            "initial_yaw_deg": "",
            "initial_yaw_source_topic": "",
            "time_normalize": "",
            "time_align": "",
            "yaw_reference": "",
            "est_time_offset_sec": "",
            "gt_time_offset_sec": "",
            "matched_samples": "",
            "rmse_3d_m": "",
            "rmse_xy_m": "",
            "rmse_3d_nobias_m": "",
            "rmse_xy_nobias_m": "",
            "mean_3d_m": "",
            "median_3d_m": "",
            "p95_3d_m": "",
            "max_3d_m": "",
            "bias_x_m": "",
            "bias_y_m": "",
            "bias_z_m": "",
            "attitude_matched_samples": "",
            "attitude_angle_rmse_deg": "",
            "attitude_angle_mean_deg": "",
            "roll_rmse_deg": "",
            "pitch_rmse_deg": "",
            "yaw_rmse_deg": "",
            "roll_bias_deg": "",
            "pitch_bias_deg": "",
            "yaw_bias_deg": "",
            "yaw_mae_deg": "",
        }

        try:
            yaw_label = (run_dir / "initial_yaw.log").read_text(encoding="utf-8")
            for line in yaw_label.splitlines():
                if line.startswith("label="):
                    metrics["initial_yaw_label"] = line.split("=", 1)[1]
                elif line.startswith("value_deg="):
                    metrics["initial_yaw_deg"] = line.split("=", 1)[1]
                elif line.startswith("topic="):
                    metrics["initial_yaw_source_topic"] = line.split("=", 1)[1]
        except Exception:
            pass

        if status == "ok":
            eval_cmd = [
                sys.executable,
                str(EVAL_SCRIPT),
                "--estimated-csv",
                str(est_csv),
                "--ground-truth-csv",
                str(gt_csv),
                "--max-time-gap-sec",
                f"{args.eval_max_time_gap_sec:.12g}",
                "--time-normalize",
                "auto",
                "--time-align",
                "auto",
                "--attitude-min-speed-mps",
                f"{args.attitude_min_speed_mps:.12g}",
                "--output-json",
                str(metrics_json),
            ]
            if att_csv.exists():
                eval_cmd.extend(["--attitude-reference-csv", str(att_csv)])
            eval_result = subprocess.run(
                eval_cmd,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            (run_dir / "evaluate.log").write_text(eval_result.stdout, encoding="utf-8")
            if eval_result.returncode == 0 and metrics_json.exists():
                payload = json.loads(metrics_json.read_text(encoding="utf-8"))
                metrics.update(payload["metrics"])
                metrics["time_normalize"] = payload.get("time_normalize", "")
                metrics["time_align"] = payload.get("time_align", "")
                metrics["yaw_reference"] = payload.get("yaw_reference", "")
                metrics["est_time_offset_sec"] = payload.get("est_time_offset_sec", "")
                metrics["gt_time_offset_sec"] = payload.get("gt_time_offset_sec", "")
                print(f"  rmse_3d_m={float(metrics['rmse_3d_m']):.6f}")
            else:
                status = f"failed_eval({eval_result.returncode})"
                print(f"  eval failed: {status}")

        row: Dict[str, object] = {"run_id": run_id, "status": status}
        for k in keys:
            row[k] = dynamic_params[k]
        row.update(metrics)
        rows.append(row)

        fieldnames = [
            "run_id",
            "status",
            *keys,
            "initial_yaw_label",
            "initial_yaw_deg",
            "initial_yaw_source_topic",
            "time_normalize",
            "time_align",
            "yaw_reference",
            "est_time_offset_sec",
            "gt_time_offset_sec",
            "matched_samples",
            "rmse_3d_m",
            "rmse_xy_m",
            "rmse_3d_nobias_m",
            "rmse_xy_nobias_m",
            "mean_3d_m",
            "median_3d_m",
            "p95_3d_m",
            "max_3d_m",
            "bias_x_m",
            "bias_y_m",
            "bias_z_m",
            "attitude_matched_samples",
            "attitude_angle_rmse_deg",
            "attitude_angle_mean_deg",
            "roll_rmse_deg",
            "pitch_rmse_deg",
            "yaw_rmse_deg",
            "roll_bias_deg",
            "pitch_bias_deg",
            "yaw_bias_deg",
            "yaw_mae_deg",
        ]
        write_summary_csv(summary_path, rows, fieldnames)

    successful_sorted: List[Dict[str, object]] = []
    successful_nobias_sorted: List[Dict[str, object]] = []
    best: Optional[Dict[str, object]] = None
    best_nobias: Optional[Dict[str, object]] = None

    successful = [
        r for r in rows if r["status"] == "ok" and isinstance(r.get("rmse_3d_m"), (int, float))
    ]
    successful_sorted = sorted(successful, key=lambda r: float(r["rmse_3d_m"]))
    if successful_sorted:
        fieldnames = [
            "run_id",
            "status",
            *keys,
            "initial_yaw_label",
            "initial_yaw_deg",
            "initial_yaw_source_topic",
            "time_normalize",
            "time_align",
            "yaw_reference",
            "est_time_offset_sec",
            "gt_time_offset_sec",
            "matched_samples",
            "rmse_3d_m",
            "rmse_xy_m",
            "rmse_3d_nobias_m",
            "rmse_xy_nobias_m",
            "mean_3d_m",
            "median_3d_m",
            "p95_3d_m",
            "max_3d_m",
            "bias_x_m",
            "bias_y_m",
            "bias_z_m",
            "attitude_matched_samples",
            "attitude_angle_rmse_deg",
            "attitude_angle_mean_deg",
            "roll_rmse_deg",
            "pitch_rmse_deg",
            "yaw_rmse_deg",
            "roll_bias_deg",
            "pitch_bias_deg",
            "yaw_bias_deg",
            "yaw_mae_deg",
        ]
        write_summary_csv(ranking_path, successful_sorted, fieldnames)

        successful_nobias = [
            r
            for r in rows
            if r["status"] == "ok"
            and isinstance(r.get("rmse_3d_nobias_m"), (int, float))
        ]
        successful_nobias_sorted = sorted(
            successful_nobias, key=lambda r: float(r["rmse_3d_nobias_m"])
        )
        if successful_nobias_sorted:
            write_summary_csv(ranking_nobias_path, successful_nobias_sorted, fieldnames)

        best = successful_sorted[0]
        best_nobias = successful_nobias_sorted[0] if successful_nobias_sorted else None
        print("\nBest run")
        print(f"  run_id: {best['run_id']}")
        print(f"  rmse_3d_m: {float(best['rmse_3d_m']):.6f}")
        best_params = {k: best[k] for k in keys}
        print(f"  params: {best_params}")

        if args.plot_best and PLOT_SCRIPT.exists():
            def run_plot(row: Dict[str, object], label: str) -> None:
                run_id = str(row.get("run_id", ""))
                if not run_id:
                    return
                run_dir = args.output_dir / run_id
                plot_time_normalize = "auto"
                plot_time_align = "auto"
                try:
                    payload = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
                    plot_time_normalize = payload.get("time_normalize", plot_time_normalize)
                    plot_time_align = payload.get("time_align", plot_time_align)
                except Exception:  # pylint: disable=broad-except
                    pass
                plot_cmd = [
                    sys.executable,
                    str(PLOT_SCRIPT),
                    "--estimated-csv",
                    str(run_dir / "estimated.csv"),
                    "--ground-truth-csv",
                    str(run_dir / "ground_truth.csv"),
                    "--output-dir",
                    str(run_dir),
                    "--prefix",
                    run_id,
                    "--time-normalize",
                    str(plot_time_normalize),
                    "--time-align",
                    str(plot_time_align),
                    "--max-time-gap-sec",
                    f"{args.eval_max_time_gap_sec:.12g}",
                    "--attitude-min-speed-mps",
                    f"{args.attitude_min_speed_mps:.12g}",
                ]
                if (run_dir / "attitude_reference.csv").exists():
                    plot_cmd.extend(["--attitude-reference-csv", str(run_dir / "attitude_reference.csv")])
                initial_yaw_plot_suffix = str(row.get("initial_yaw_label", ""))
                if initial_yaw_plot_suffix:
                    plot_cmd.extend(["--title-suffix", initial_yaw_plot_suffix])

                plot_result = subprocess.run(
                    plot_cmd,
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                (run_dir / "plot_pose_csv.log").write_text(plot_result.stdout, encoding="utf-8")
                if plot_result.returncode == 0:
                    print(f"{label}_plots: {run_dir}")
                else:
                    print(
                        f"  warning: plot_pose_csv failed for {label} (code={plot_result.returncode})"
                    )

            plotted_run_ids = set()
            run_plot(best, "best_run")
            plotted_run_ids.add(str(best.get("run_id", "")))
            if best_nobias is not None and str(best_nobias.get("run_id", "")) not in plotted_run_ids:
                run_plot(best_nobias, "best_nobias_run")
    else:
        print("\nNo successful evaluations.")

    print(f"summary_csv: {summary_path}")
    if ranking_path.exists():
        print(f"ranking_csv: {ranking_path}")
    if ranking_nobias_path.exists():
        print(f"ranking_nobias_csv: {ranking_nobias_path}")
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = write_html_report(
        output_dir=args.output_dir,
        stamp=stamp,
        bag_path=args.bag_path,
        param_grid_json=args.param_grid_json,
        summary_csv=summary_path,
        ranking_csv=ranking_path,
        ranking_nobias_csv=ranking_nobias_path,
        best=best,
        best_nobias=best_nobias,
        keys=keys,
        successful_sorted=successful_sorted,
        successful_nobias_sorted=successful_nobias_sorted,
    )
    print(f"report_html: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
