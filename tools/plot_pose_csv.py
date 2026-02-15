#!/usr/bin/env python3
"""Plot estimated vs reference pose CSV.

This script generates:
- XY trajectory plot with START/GOAL markers.
- Time series plot for z + RPY, including reference yaw (from GT quaternion or course).

CSV format is compatible with tools/record_pose_csv.py:
  t_sec,x,y,z,qx,qy,qz,qw
"""

from __future__ import annotations

import argparse
import csv
import math
import warnings
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
# Matplotlib can warn about Axes3D availability depending on installation.
warnings.filterwarnings("ignore", message=r"Unable to import Axes3D\..*")
import matplotlib.pyplot as plt  # noqa: E402  pylint: disable=wrong-import-position


@dataclass(frozen=True)
class PoseSample:
    t_sec: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def wrap_to_pi(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not (n > 0.0) or not math.isfinite(n):
        return float("nan"), float("nan"), float("nan")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if sinp >= 1.0:
        pitch = math.pi / 2.0
    elif sinp <= -1.0:
        pitch = -math.pi / 2.0
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def read_pose_csv(csv_path: Path, min_time_sec: float) -> List[PoseSample]:
    rows: List[PoseSample] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        rd = csv.DictReader(f)
        if rd.fieldnames is None:
            raise ValueError(f"{csv_path} has no header")
        required = {"t_sec", "x", "y", "z"}
        if not required.issubset(set(rd.fieldnames)):
            raise ValueError(f"{csv_path} missing required columns: {sorted(required)}")

        has_quat = {"qx", "qy", "qz", "qw"}.issubset(set(rd.fieldnames))

        for row in rd:
            try:
                t = float(row["t_sec"])
                if t < min_time_sec:
                    continue
                x = float(row["x"])
                y = float(row["y"])
                z = float(row["z"])
                if has_quat:
                    qx = float(row["qx"])
                    qy = float(row["qy"])
                    qz = float(row["qz"])
                    qw = float(row["qw"])
                else:
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            except (TypeError, ValueError):
                continue

            if not all(math.isfinite(v) for v in (t, x, y, z, qx, qy, qz, qw)):
                continue
            rows.append(PoseSample(t_sec=t, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))

    rows.sort(key=lambda s: s.t_sec)
    return rows


def filter_pose_samples(
    samples: Sequence[PoseSample], *, drop_stamp_zero: bool, stamp_zero_abs_tol: float
) -> List[PoseSample]:
    filtered: List[PoseSample] = []
    for s in samples:
        if drop_stamp_zero and math.isclose(s.t_sec, 0.0, rel_tol=0.0, abs_tol=stamp_zero_abs_tol):
            continue
        filtered.append(s)
    return filtered


def gt_quat_is_identity(gt: Sequence[PoseSample], tol: float = 1e-6) -> bool:
    if not gt:
        return True
    num_identity = 0
    for s in gt:
        if abs(s.qw - 1.0) < tol and abs(s.qx) < tol and abs(s.qy) < tol and abs(s.qz) < tol:
            num_identity += 1
    return (num_identity / len(gt)) > 0.95


def match_segment_at_time(
    samples: Sequence[PoseSample], times: Sequence[float], t_sec: float
) -> Optional[Tuple[PoseSample, PoseSample, float, float]]:
    if len(samples) < 2:
        return None

    idx = bisect_left(times, t_sec)

    if idx == 0:
        return None
    if idx >= len(samples):
        return None

    left = samples[idx - 1]
    right = samples[idx]
    gap_left = t_sec - left.t_sec
    gap_right = right.t_sec - t_sec
    if gap_left < 0.0 or gap_right < 0.0:
        return None
    return left, right, gap_left, gap_right


def plot_xy_trajectory(
    est: Sequence[PoseSample],
    gt: Sequence[PoseSample],
    out_path: Path,
    title: str,
) -> None:
    def xs(samples):
        return [s.x for s in samples]

    def ys(samples):
        return [s.y for s in samples]

    est_x, est_y = xs(est), ys(est)
    gt_x, gt_y = xs(gt), ys(gt)

    plt.figure(figsize=(11, 9))
    plt.plot(gt_x, gt_y, linewidth=1.8, color="#1f77b4", label="ground_truth")
    plt.plot(est_x, est_y, linewidth=1.4, color="#d62728", label="estimated")

    def annotate_start_goal(x, y, color, prefix):
        if not x:
            return
        plt.scatter([x[0]], [y[0]], s=220, marker="o", color=color, edgecolors="black", zorder=5)
        plt.annotate(
            f"{prefix} START",
            (x[0], y[0]),
            textcoords="offset points",
            xytext=(10, 10),
            fontsize=10,
            fontweight="bold",
            color=color,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color, alpha=0.9),
        )

        plt.scatter([x[-1]], [y[-1]], s=280, marker="*", color=color, edgecolors="black", zorder=6)
        plt.annotate(
            f"{prefix} GOAL",
            (x[-1], y[-1]),
            textcoords="offset points",
            xytext=(10, -18),
            fontsize=10,
            fontweight="bold",
            color=color,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec=color, alpha=0.9),
        )

        if len(x) > 20:
            plt.annotate(
                "",
                xy=(x[20], y[20]),
                xytext=(x[0], y[0]),
                arrowprops=dict(arrowstyle="->", lw=2.0, color=color, alpha=0.9),
            )

    annotate_start_goal(gt_x, gt_y, "#1f77b4", "GT")
    annotate_start_goal(est_x, est_y, "#d62728", "EST")

    plt.title(title)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.35)
    plt.legend(loc="best")
    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=170)
    plt.close()


def plot_timeseries_z_rpy(
    est: Sequence[PoseSample],
    gt: Sequence[PoseSample],
    out_path: Path,
    title: str,
    yaw_reference: str,
    max_time_gap_sec: float,
) -> None:
    if not est or not gt:
        raise RuntimeError("empty CSV")
    if max_time_gap_sec <= 0.0:
        raise ValueError("--max-time-gap-sec must be > 0")

    t0 = min(est[0].t_sec, gt[0].t_sec)
    est_t = [s.t_sec - t0 for s in est]
    gt_t = [s.t_sec - t0 for s in gt]

    est_z = [s.z for s in est]
    gt_z = [s.z for s in gt]

    est_r, est_p, est_yaw = zip(*(quat_to_rpy(s.qx, s.qy, s.qz, s.qw) for s in est))
    est_r_d = [v * 180.0 / math.pi for v in est_r]
    est_p_d = [v * 180.0 / math.pi for v in est_p]
    est_yaw_d = [v * 180.0 / math.pi for v in est_yaw]

    has_gt_quat = not gt_quat_is_identity(gt)
    gt_r_d: List[float] = []
    gt_p_d: List[float] = []
    gt_yaw_d: List[float] = []
    if has_gt_quat:
        gt_r, gt_p, gt_yaw = zip(*(quat_to_rpy(s.qx, s.qy, s.qz, s.qw) for s in gt))
        gt_r_d = [v * 180.0 / math.pi for v in gt_r]
        gt_p_d = [v * 180.0 / math.pi for v in gt_p]
        gt_yaw_d = [v * 180.0 / math.pi for v in gt_yaw]

    # Build reference yaw and yaw error on EST timestamps.
    ref_yaw_deg: List[float] = []
    err_yaw_deg: List[float] = []
    err_t: List[float] = []

    if yaw_reference == "gt_quat" and not has_gt_quat:
        yaw_reference = "gt_course"

    gt_times = [s.t_sec for s in gt]
    for s, t_rel, yaw_est in zip(est, est_t, est_yaw):
        seg = match_segment_at_time(gt, gt_times, s.t_sec)
        if seg is None:
            continue
        left, right, gap_left, gap_right = seg
        if min(gap_left, gap_right) > max_time_gap_sec:
            continue

        if yaw_reference == "gt_quat" and has_gt_quat:
            _, _, yaw_ref = quat_to_rpy(left.qx, left.qy, left.qz, left.qw)
        else:
            dx = right.x - left.x
            dy = right.y - left.y
            if abs(dx) < 1e-12 and abs(dy) < 1e-12:
                continue
            yaw_ref = math.atan2(dy, dx)

        ref_yaw_deg.append(yaw_ref * 180.0 / math.pi)
        err_yaw_deg.append(wrap_to_pi(yaw_est - yaw_ref) * 180.0 / math.pi)
        err_t.append(t_rel)

    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True)

    axes[0].plot(gt_t, gt_z, color="#1f77b4", linewidth=1.4, label="GT z")
    axes[0].plot(est_t, est_z, color="#d62728", linewidth=1.0, label="EST z")
    axes[0].set_ylabel("z [m]")
    axes[0].grid(True, linestyle="--", alpha=0.35)
    axes[0].legend(loc="best")

    if has_gt_quat:
        axes[1].plot(gt_t, gt_r_d, color="#1f77b4", linewidth=1.2, label="GT roll")
    axes[1].plot(est_t, est_r_d, color="#d62728", linewidth=1.0, label="EST roll")
    axes[1].set_ylabel("roll [deg]")
    axes[1].grid(True, linestyle="--", alpha=0.35)
    axes[1].legend(loc="best")

    if has_gt_quat:
        axes[2].plot(gt_t, gt_p_d, color="#1f77b4", linewidth=1.2, label="GT pitch")
    axes[2].plot(est_t, est_p_d, color="#d62728", linewidth=1.0, label="EST pitch")
    axes[2].set_ylabel("pitch [deg]")
    axes[2].grid(True, linestyle="--", alpha=0.35)
    axes[2].legend(loc="best")

    axes[3].plot(est_t, est_yaw_d, color="#d62728", linewidth=1.0, label="EST yaw")
    if yaw_reference == "gt_quat" and has_gt_quat:
        axes[3].plot(gt_t, gt_yaw_d, color="#1f77b4", linewidth=1.2, label="GT yaw")
    else:
        axes[3].plot(err_t, ref_yaw_deg, color="#1f77b4", linewidth=1.2, label="REF yaw (course)")
    axes[3].set_ylabel("yaw [deg]")
    axes[3].grid(True, linestyle="--", alpha=0.35)
    axes[3].legend(loc="best")

    axes[4].plot(err_t, err_yaw_deg, color="black", linewidth=1.0, label="yaw error (EST-REF)")
    axes[4].set_ylabel("yaw err [deg]")
    axes[4].set_xlabel("t [sec] (offset)")
    axes[4].grid(True, linestyle="--", alpha=0.35)
    axes[4].legend(loc="best")

    fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=[0, 0.02, 1, 0.98])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170)
    plt.close(fig)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--estimated-csv", required=True, type=Path)
    p.add_argument("--ground-truth-csv", required=True, type=Path)
    p.add_argument("--output-dir", type=Path, default=Path("."))
    p.add_argument("--prefix", default="plot")
    p.add_argument("--min-time-sec", type=float, default=0.0, help="drop samples with t_sec < this")
    p.add_argument(
        "--keep-zero-stamp",
        action="store_true",
        help="keep samples whose t_sec is exactly 0.0 (default: drop them)",
    )
    p.add_argument("--max-time-gap-sec", type=float, default=0.1, help="time gate for yaw reference/error")
    p.add_argument(
        "--yaw-reference",
        choices=["auto", "gt_quat", "gt_course"],
        default="auto",
        help="reference yaw source. auto prefers gt_quat if available, otherwise gt_course",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    est = read_pose_csv(args.estimated_csv, min_time_sec=args.min_time_sec)
    gt = read_pose_csv(args.ground_truth_csv, min_time_sec=args.min_time_sec)
    est = filter_pose_samples(est, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12)
    gt = filter_pose_samples(gt, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12)

    yaw_reference = args.yaw_reference
    if yaw_reference == "auto":
        yaw_reference = "gt_course" if gt_quat_is_identity(gt) else "gt_quat"

    xy_path = args.output_dir / f"{args.prefix}_trajectory_xy.png"
    ts_path = args.output_dir / f"{args.prefix}_timeseries_z_rpy.png"

    plot_xy_trajectory(
        est=est,
        gt=gt,
        out_path=xy_path,
        title=f"Trajectory XY ({args.prefix})",
    )
    plot_timeseries_z_rpy(
        est=est,
        gt=gt,
        out_path=ts_path,
        title=f"Time Series z+RPY ({args.prefix})",
        yaw_reference=yaw_reference,
        max_time_gap_sec=args.max_time_gap_sec,
    )

    print(xy_path)
    print(ts_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
