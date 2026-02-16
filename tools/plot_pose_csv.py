#!/usr/bin/env python3
"""Plot estimated vs reference pose CSV.

This script generates:
- XY trajectory plot with START/GOAL markers.
- Time series plot for z + RPY, including reference yaw (from GT quaternion, attitude reference, or course).

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


UNIX_TO_GPS_EPOCH_OFFSET_SEC = 315964800.0  # 1970-01-01 -> 1980-01-06
GPS_WEEK_SEC = 604800.0


def normalize_unix_to_gps_tow(samples: Sequence[PoseSample], gps_leap_seconds: int) -> List[PoseSample]:
    """Convert unix epoch stamps to GPS time-of-week (TOW) (heuristically)."""
    if not samples:
        return []
    if not isinstance(gps_leap_seconds, int):
        raise ValueError("--gps-leap-seconds must be an integer")

    out: List[PoseSample] = []
    for s in samples:
        t_sec = s.t_sec
        if t_sec > 1.0e8:  # unix epoch seconds are ~1e9+
            t_sec = (t_sec - UNIX_TO_GPS_EPOCH_OFFSET_SEC + float(gps_leap_seconds)) % GPS_WEEK_SEC
        out.append(
            PoseSample(
                t_sec=t_sec,
                x=s.x,
                y=s.y,
                z=s.z,
                qx=s.qx,
                qy=s.qy,
                qz=s.qz,
                qw=s.qw,
            )
        )
    out.sort(key=lambda v: v.t_sec)
    return out


def apply_time_normalize(
    samples: Sequence[PoseSample], mode: str, gps_leap_seconds: int
) -> List[PoseSample]:
    if mode == "none":
        return list(samples)
    if mode == "unix_to_gps_tow":
        return normalize_unix_to_gps_tow(samples, gps_leap_seconds=gps_leap_seconds)
    raise ValueError(f"unsupported time normalize mode: {mode}")


def shift_time(samples: Sequence[PoseSample], offset_sec: float) -> List[PoseSample]:
    if not samples:
        return []
    if not (math.isfinite(offset_sec) and offset_sec != 0.0):
        return list(samples)
    return [
        PoseSample(
            t_sec=s.t_sec - offset_sec,
            x=s.x,
            y=s.y,
            z=s.z,
            qx=s.qx,
            qy=s.qy,
            qz=s.qz,
            qw=s.qw,
        )
        for s in samples
    ]


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


def count_time_matches(
    est: Sequence[PoseSample], gt: Sequence[PoseSample], max_time_gap_sec: float
) -> int:
    if not est or len(gt) < 2:
        return 0
    gt_times = [s.t_sec for s in gt]
    count = 0
    for s in est:
        seg = match_segment_at_time(gt, gt_times, s.t_sec)
        if seg is None:
            continue
        _, _, gap_left, gap_right = seg
        if min(gap_left, gap_right) <= max_time_gap_sec:
            count += 1
    return count


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
    attitude_ref: Optional[Sequence[PoseSample]],
    out_path: Path,
    title: str,
    yaw_reference: str,
    rpy_reference: str,
    max_time_gap_sec: float,
) -> None:
    if not est or not gt:
        raise RuntimeError("empty CSV")
    if max_time_gap_sec <= 0.0:
        raise ValueError("--max-time-gap-sec must be > 0")

    t0 = min(
        est[0].t_sec,
        gt[0].t_sec,
        attitude_ref[0].t_sec if attitude_ref else float("inf"),
    )
    est_t = [s.t_sec - t0 for s in est]
    gt_t = [s.t_sec - t0 for s in gt]
    ref_t: List[float] = []
    if attitude_ref:
        ref_t = [s.t_sec - t0 for s in attitude_ref]

    est_z = [s.z for s in est]
    gt_z = [s.z for s in gt]

    est_r, est_p, est_yaw = zip(*(quat_to_rpy(s.qx, s.qy, s.qz, s.qw) for s in est))
    est_r_d = [v * 180.0 / math.pi for v in est_r]
    est_p_d = [v * 180.0 / math.pi for v in est_p]
    est_yaw_d = [v * 180.0 / math.pi for v in est_yaw]

    has_att_quat = bool(attitude_ref) and not gt_quat_is_identity(attitude_ref or [])
    has_gt_quat = not gt_quat_is_identity(gt)

    # Reference for roll/pitch display (cannot be derived from course).
    rpy_ref_quat: Optional[Sequence[PoseSample]] = None
    rpy_ref_t: List[float] = []
    rpy_ref_label = ""
    if rpy_reference == "auto":
        # Prefer GT quaternion when available (true attitude).
        rpy_reference = "gt_quat" if has_gt_quat else ("attitude_csv" if has_att_quat else "gt_quat")
    if rpy_reference == "gt_quat" and has_gt_quat:
        rpy_ref_quat = gt
        rpy_ref_t = gt_t
        rpy_ref_label = "GT"
    elif rpy_reference == "attitude_csv" and has_att_quat:
        rpy_ref_quat = attitude_ref
        rpy_ref_t = ref_t
        rpy_ref_label = "REF(attitude_csv)"

    rpy_ref_r_d: List[float] = []
    rpy_ref_p_d: List[float] = []
    rpy_ref_yaw_d: List[float] = []
    if rpy_ref_quat is not None:
        rpy_ref_r, rpy_ref_p, rpy_ref_yaw = zip(
            *(quat_to_rpy(s.qx, s.qy, s.qz, s.qw) for s in rpy_ref_quat)
        )
        rpy_ref_r_d = [v * 180.0 / math.pi for v in rpy_ref_r]
        rpy_ref_p_d = [v * 180.0 / math.pi for v in rpy_ref_p]
        rpy_ref_yaw_d = [v * 180.0 / math.pi for v in rpy_ref_yaw]

    # Build reference yaw and angle errors on EST timestamps.
    course_yaw_deg: List[float] = []
    err_roll_deg: List[float] = []
    err_pitch_deg: List[float] = []
    err_yaw_deg: List[float] = []
    err_t: List[float] = []

    # Resolve yaw reference.
    if yaw_reference == "attitude_csv" and not has_att_quat:
        yaw_reference = "gt_quat" if has_gt_quat else "gt_course"
    if yaw_reference == "gt_quat" and not has_gt_quat:
        yaw_reference = "attitude_csv" if has_att_quat else "gt_course"

    yaw_ref_quat: Optional[Sequence[PoseSample]] = None
    yaw_ref_t: List[float] = []
    yaw_ref_label = ""
    yaw_ref_yaw_d: List[float] = []
    if yaw_reference == "attitude_csv" and has_att_quat:
        yaw_ref_quat = attitude_ref
        yaw_ref_t = ref_t
        yaw_ref_label = "REF(attitude_csv)"
    elif yaw_reference == "gt_quat" and has_gt_quat:
        yaw_ref_quat = gt
        yaw_ref_t = gt_t
        yaw_ref_label = "GT"
    if yaw_ref_quat is not None:
        _, _, yaw_ref_yaw = zip(*(quat_to_rpy(s.qx, s.qy, s.qz, s.qw) for s in yaw_ref_quat))
        yaw_ref_yaw_d = [v * 180.0 / math.pi for v in yaw_ref_yaw]

    gt_times = [s.t_sec for s in gt]
    yaw_ref_times: List[float] = []
    if yaw_ref_quat is not None:
        yaw_ref_times = [s.t_sec for s in yaw_ref_quat]

    for s, t_rel, yaw_est in zip(est, est_t, est_yaw):
        yaw_ref: Optional[float] = None

        if yaw_reference in ("gt_quat", "attitude_csv") and yaw_ref_quat is not None:
            seg = match_segment_at_time(yaw_ref_quat, yaw_ref_times, s.t_sec)
            if seg is None:
                continue
            left, right, gap_left, gap_right = seg
            if min(gap_left, gap_right) > max_time_gap_sec:
                continue
            ref_sample = left if gap_left <= gap_right else right
            roll_ref, pitch_ref, yaw_ref_tmp = quat_to_rpy(
                ref_sample.qx, ref_sample.qy, ref_sample.qz, ref_sample.qw
            )
            roll_est, pitch_est, _ = quat_to_rpy(s.qx, s.qy, s.qz, s.qw)
            if not all(math.isfinite(v) for v in (roll_ref, pitch_ref, yaw_ref_tmp, roll_est, pitch_est)):
                continue

            err_roll_deg.append(wrap_to_pi(roll_est - roll_ref) * 180.0 / math.pi)
            err_pitch_deg.append(wrap_to_pi(pitch_est - pitch_ref) * 180.0 / math.pi)
            yaw_ref = yaw_ref_tmp

        else:
            seg = match_segment_at_time(gt, gt_times, s.t_sec)
            if seg is None:
                continue
            left, right, gap_left, gap_right = seg
            if min(gap_left, gap_right) > max_time_gap_sec:
                continue
            dx = right.x - left.x
            dy = right.y - left.y
            if abs(dx) < 1e-12 and abs(dy) < 1e-12:
                continue
            yaw_ref = math.atan2(dy, dx)
            course_yaw_deg.append(yaw_ref * 180.0 / math.pi)

        if yaw_ref is None or not math.isfinite(yaw_ref):
            continue

        err_yaw_deg.append(wrap_to_pi(yaw_est - yaw_ref) * 180.0 / math.pi)
        err_t.append(t_rel)

    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True)

    axes[0].plot(gt_t, gt_z, color="#1f77b4", linewidth=1.4, label="GT z")
    axes[0].plot(est_t, est_z, color="#d62728", linewidth=1.0, label="EST z")
    axes[0].set_ylabel("z [m]")
    axes[0].grid(True, linestyle="--", alpha=0.35)
    axes[0].legend(loc="best")

    if rpy_ref_quat is not None:
        axes[1].plot(
            rpy_ref_t,
            rpy_ref_r_d,
            color="#1f77b4",
            linewidth=1.2,
            label=f"{rpy_ref_label} roll",
        )
    axes[1].plot(est_t, est_r_d, color="#d62728", linewidth=1.0, label="EST roll")
    axes[1].set_ylabel("roll [deg]")
    axes[1].grid(True, linestyle="--", alpha=0.35)
    axes[1].legend(loc="best")

    if rpy_ref_quat is not None:
        axes[2].plot(
            rpy_ref_t,
            rpy_ref_p_d,
            color="#1f77b4",
            linewidth=1.2,
            label=f"{rpy_ref_label} pitch",
        )
    axes[2].plot(est_t, est_p_d, color="#d62728", linewidth=1.0, label="EST pitch")
    axes[2].set_ylabel("pitch [deg]")
    axes[2].grid(True, linestyle="--", alpha=0.35)
    axes[2].legend(loc="best")

    axes[3].plot(est_t, est_yaw_d, color="#d62728", linewidth=1.0, label="EST yaw")
    if yaw_reference in ("gt_quat", "attitude_csv") and yaw_ref_quat is not None:
        axes[3].plot(
            yaw_ref_t,
            yaw_ref_yaw_d,
            color="#1f77b4",
            linewidth=1.2,
            label=f"{yaw_ref_label} yaw",
        )
    else:
        axes[3].plot(err_t, course_yaw_deg, color="#1f77b4", linewidth=1.2, label="REF yaw (course)")
    axes[3].set_ylabel("yaw [deg]")
    axes[3].grid(True, linestyle="--", alpha=0.35)
    axes[3].legend(loc="best")

    if yaw_reference in ("gt_quat", "attitude_csv") and yaw_ref_quat is not None:
        axes[4].plot(err_t, err_roll_deg, color="#2ca02c", linewidth=1.0, label="roll error (EST-REF)")
        axes[4].plot(err_t, err_pitch_deg, color="#ff7f0e", linewidth=1.0, label="pitch error (EST-REF)")
        axes[4].plot(err_t, err_yaw_deg, color="black", linewidth=1.0, label="yaw error (EST-REF)")
        axes[4].set_ylabel("angle err [deg]")
    else:
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
    p.add_argument(
        "--attitude-reference-csv",
        type=Path,
        default=None,
        help="optional attitude reference (Pose CSV with quaternion, e.g. recorded from sensor_msgs/Imu)",
    )
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
        "--time-normalize",
        choices=["none", "unix_to_gps_tow", "auto"],
        default="none",
        help=(
            "optional timestamp normalization. "
            "'unix_to_gps_tow' converts unix epoch stamps to GPS time-of-week. "
            "'auto' tries both none/unix_to_gps_tow and selects the best."
        ),
    )
    p.add_argument(
        "--gps-leap-seconds",
        type=int,
        default=18,
        help="GPS leap seconds used for unix_to_gps_tow conversion (default: 18).",
    )
    p.add_argument(
        "--time-align",
        choices=["absolute", "relative", "auto"],
        default="auto",
        help=(
            "time alignment mode before matching reference yaw. "
            "'relative' subtracts each CSV's first timestamp. "
            "'auto' selects absolute/relative based on which yields more matches."
        ),
    )
    p.add_argument(
        "--yaw-reference",
        choices=["auto", "gt_quat", "gt_course", "attitude_csv"],
        default="auto",
        help="reference yaw source. auto prefers gt_quat (if available), then attitude_csv, otherwise gt_course",
    )
    p.add_argument(
        "--rpy-reference",
        choices=["auto", "gt_quat", "attitude_csv"],
        default="auto",
        help="reference source for roll/pitch display. auto prefers gt_quat (if available), otherwise attitude_csv",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    est = read_pose_csv(args.estimated_csv, min_time_sec=args.min_time_sec)
    gt = read_pose_csv(args.ground_truth_csv, min_time_sec=args.min_time_sec)
    attitude_ref: Optional[List[PoseSample]] = None
    if args.attitude_reference_csv is not None:
        attitude_ref = read_pose_csv(args.attitude_reference_csv, min_time_sec=args.min_time_sec)
    est = filter_pose_samples(est, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12)
    gt = filter_pose_samples(gt, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12)
    if attitude_ref is not None:
        attitude_ref = filter_pose_samples(
            attitude_ref, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
        )

    time_align_candidates = ["absolute", "relative"] if args.time_align == "auto" else [args.time_align]
    time_normalize_candidates = (
        ["none", "unix_to_gps_tow"]
        if args.time_normalize == "auto"
        else [args.time_normalize]
    )

    best_matches = -1
    used_time_align = "absolute"
    used_time_normalize = "none"
    best_est = list(est)
    best_gt = list(gt)
    best_att = list(attitude_ref) if attitude_ref is not None else None

    for time_normalize in time_normalize_candidates:
        est_norm = apply_time_normalize(
            est, mode=time_normalize, gps_leap_seconds=args.gps_leap_seconds
        )
        gt_norm = apply_time_normalize(
            gt, mode=time_normalize, gps_leap_seconds=args.gps_leap_seconds
        )
        att_norm = (
            apply_time_normalize(attitude_ref, mode=time_normalize, gps_leap_seconds=args.gps_leap_seconds)
            if attitude_ref is not None
            else None
        )
        for time_align in time_align_candidates:
            est_aligned = est_norm
            gt_aligned = gt_norm
            att_aligned = att_norm
            if time_align == "relative":
                if est_norm:
                    est_aligned = shift_time(est_norm, est_norm[0].t_sec)
                if gt_norm:
                    gt_aligned = shift_time(gt_norm, gt_norm[0].t_sec)
                if att_norm:
                    att_aligned = shift_time(att_norm, att_norm[0].t_sec)

            matches = count_time_matches(est_aligned, gt_aligned, args.max_time_gap_sec)

            better = False
            if matches > best_matches:
                better = True
            elif matches == best_matches:
                # Prefer smaller transformations when equivalent.
                if used_time_align == "relative" and time_align == "absolute":
                    better = True
                elif used_time_align == time_align:
                    if used_time_normalize == "unix_to_gps_tow" and time_normalize == "none":
                        better = True

            if better:
                best_matches = matches
                used_time_align = time_align
                used_time_normalize = time_normalize
                best_est = list(est_aligned)
                best_gt = list(gt_aligned)
                best_att = list(att_aligned) if att_aligned is not None else None

    est = best_est
    gt = best_gt
    attitude_ref = best_att

    has_att_quat = attitude_ref is not None and not gt_quat_is_identity(attitude_ref)
    has_gt_quat = not gt_quat_is_identity(gt)

    yaw_reference = args.yaw_reference
    if yaw_reference == "auto":
        # Prefer GT quaternion when available (true attitude), otherwise fall back
        # to recorded attitude reference, otherwise use course from GT positions.
        if has_gt_quat:
            yaw_reference = "gt_quat"
        elif has_att_quat:
            yaw_reference = "attitude_csv"
        else:
            yaw_reference = "gt_course"

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
        attitude_ref=attitude_ref,
        out_path=ts_path,
        title=f"Time Series z+RPY ({args.prefix})",
        yaw_reference=yaw_reference,
        rpy_reference=args.rpy_reference,
        max_time_gap_sec=args.max_time_gap_sec,
    )

    print(xy_path)
    print(ts_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
