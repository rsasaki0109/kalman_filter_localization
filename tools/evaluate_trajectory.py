#!/usr/bin/env python3
"""Compute trajectory error metrics from estimated and ground-truth CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    t_sec: float
    x: float
    y: float
    z: float
    # Optional orientation (record_pose_csv.py always writes it, but some external CSVs may not).
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


UNIX_TO_GPS_EPOCH_OFFSET_SEC = 315964800.0  # 1970-01-01 -> 1980-01-06
GPS_WEEK_SEC = 604800.0


def normalize_unix_to_gps_tow(samples: Sequence[PoseSample], gps_leap_seconds: int) -> List[PoseSample]:
    """Convert unix epoch stamps to GPS time-of-week (TOW).

    This is useful for open datasets where some topics are stamped in unix epoch time and others
    are stamped using GPS time (week, TOW). We only convert values that look like unix epoch
    seconds, leaving already-small stamps untouched.
    """
    if not samples:
        return []
    if not isinstance(gps_leap_seconds, int):
        raise ValueError("--gps-leap-seconds must be an integer")

    out: List[PoseSample] = []
    for s in samples:
        t_sec = s.t_sec
        if t_sec > 1.0e8:  # heuristic: unix epoch seconds are ~1e9+
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


def filter_pose_samples(
    samples: Sequence[PoseSample], *, drop_stamp_zero: bool, stamp_zero_abs_tol: float
) -> List[PoseSample]:
    filtered: List[PoseSample] = []
    for s in samples:
        if drop_stamp_zero and math.isclose(s.t_sec, 0.0, rel_tol=0.0, abs_tol=stamp_zero_abs_tol):
            continue
        filtered.append(s)
    return filtered


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


def read_pose_csv(
    csv_path: Path,
    time_col: str,
    x_col: str,
    y_col: str,
    z_col: str,
    time_scale: float,
) -> List[PoseSample]:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    if not (time_scale > 0.0 and math.isfinite(time_scale)):
        raise ValueError(f"time_scale must be finite and > 0.0, got {time_scale}")

    rows: List[PoseSample] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {time_col, x_col, y_col, z_col}
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} has no header")
        missing = required.difference(reader.fieldnames)
        if missing:
            missing_text = ", ".join(sorted(missing))
            raise ValueError(f"{csv_path} missing required columns: {missing_text}")

        has_quat = {"qx", "qy", "qz", "qw"}.issubset(set(reader.fieldnames))

        for line_no, row in enumerate(reader, start=2):
            try:
                t_raw = float(row[time_col])
                x = float(row[x_col])
                y = float(row[y_col])
                z = float(row[z_col])
                if has_quat:
                    qx = float(row["qx"])
                    qy = float(row["qy"])
                    qz = float(row["qz"])
                    qw = float(row["qw"])
                else:
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            except (TypeError, ValueError) as e:
                raise ValueError(
                    f"{csv_path}:{line_no} contains non-numeric values for "
                    f"columns {time_col},{x_col},{y_col},{z_col}"
                ) from e

            t_sec = t_raw * time_scale
            if not all(math.isfinite(v) for v in (t_sec, x, y, z, qx, qy, qz, qw)):
                continue
            rows.append(PoseSample(t_sec=t_sec, x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))

    rows.sort(key=lambda s: s.t_sec)
    return rows


def match_pose_at_time(
    samples: Sequence[PoseSample], times: Sequence[float], t_sec: float
) -> Optional[Tuple[PoseSample, float, float]]:
    if len(samples) == 0:
        return None
    if len(samples) == 1:
        if math.isclose(samples[0].t_sec, t_sec, rel_tol=0.0, abs_tol=1e-12):
            return samples[0], 0.0, 0.0
        return None

    idx = bisect_left(times, t_sec)

    if idx < len(samples) and math.isclose(samples[idx].t_sec, t_sec, rel_tol=0.0, abs_tol=1e-12):
        matched = samples[idx]
        return matched, 0.0, 0.0

    if idx <= 0:
        return None
    if idx >= len(samples):
        if math.isclose(samples[-1].t_sec, t_sec, rel_tol=0.0, abs_tol=1e-12):
            matched = samples[-1]
            return matched, 0.0, 0.0
        return None

    left = samples[idx - 1]
    right = samples[idx]
    dt = right.t_sec - left.t_sec
    if dt <= 0.0:
        return None

    alpha = (t_sec - left.t_sec) / dt
    interp = PoseSample(
        t_sec=t_sec,
        x=(1.0 - alpha) * left.x + alpha * right.x,
        y=(1.0 - alpha) * left.y + alpha * right.y,
        z=(1.0 - alpha) * left.z + alpha * right.z,
    )
    return interp, t_sec - left.t_sec, right.t_sec - t_sec


def quat_sequence_is_identity(samples: Sequence[PoseSample], tol: float = 1e-6) -> bool:
    if not samples:
        return True
    num_identity = 0
    for s in samples:
        if abs(s.qw - 1.0) < tol and abs(s.qx) < tol and abs(s.qy) < tol and abs(s.qz) < tol:
            num_identity += 1
    return (num_identity / len(samples)) > 0.95


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


def quat_angle_error_deg(est: PoseSample, ref: PoseSample) -> Optional[float]:
    """Shortest rotation angle between two quaternions in degrees."""

    def normalize(qx: float, qy: float, qz: float, qw: float) -> Optional[Tuple[float, float, float, float]]:
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not (n > 0.0) or not math.isfinite(n):
            return None
        return qx / n, qy / n, qz / n, qw / n

    q_est = normalize(est.qx, est.qy, est.qz, est.qw)
    q_ref = normalize(ref.qx, ref.qy, ref.qz, ref.qw)
    if q_est is None or q_ref is None:
        return None

    ex, ey, ez, ew = q_est
    rx, ry, rz, rw = q_ref
    dot = ex * rx + ey * ry + ez * rz + ew * rw
    dot = max(-1.0, min(1.0, abs(dot)))
    angle_rad = 2.0 * math.acos(dot)
    return angle_rad * 180.0 / math.pi


def rmse(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("rmse() requires at least one value")
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def compute_attitude_metrics(
    est: Sequence[PoseSample],
    gt: Sequence[PoseSample],
    attitude_ref: Optional[Sequence[PoseSample]],
    *,
    max_time_gap_sec: float,
    yaw_reference: str,
    attitude_min_speed_mps: float,
) -> Dict[str, float]:
    """Compute optional attitude metrics based on EST quaternion and a chosen reference."""
    if not est or not gt:
        return {}
    if max_time_gap_sec <= 0.0 or not math.isfinite(max_time_gap_sec):
        raise ValueError("max_time_gap_sec must be finite and > 0.0")
    if attitude_min_speed_mps < 0.0 or not math.isfinite(attitude_min_speed_mps):
        raise ValueError("attitude_min_speed_mps must be finite and >= 0.0")

    out: Dict[str, float] = {}

    gt_times = [s.t_sec for s in gt]

    def gt_speed_xy_mps(t_sec: float) -> Optional[float]:
        seg = match_segment_at_time(gt, gt_times, t_sec)
        if seg is None:
            return None
        left, right, gap_left, gap_right = seg
        if min(gap_left, gap_right) > max_time_gap_sec:
            return None
        dt = right.t_sec - left.t_sec
        if dt <= 0.0:
            return None
        dist_xy = math.hypot(right.x - left.x, right.y - left.y)
        return dist_xy / dt

    if yaw_reference in ("gt_quat", "attitude_csv"):
        ref = gt if yaw_reference == "gt_quat" else (attitude_ref or [])
        if len(ref) < 2:
            return {}
        ref_times = [s.t_sec for s in ref]

        err_roll_deg: List[float] = []
        err_pitch_deg: List[float] = []
        err_yaw_deg: List[float] = []
        err_angle_deg: List[float] = []

        for s in est:
            if attitude_min_speed_mps > 0.0:
                speed_xy = gt_speed_xy_mps(s.t_sec)
                if speed_xy is None or speed_xy < attitude_min_speed_mps:
                    continue
            seg = match_segment_at_time(ref, ref_times, s.t_sec)
            if seg is None:
                continue
            left, right, gap_left, gap_right = seg
            if min(gap_left, gap_right) > max_time_gap_sec:
                continue
            ref_s = left if gap_left <= gap_right else right

            roll_ref, pitch_ref, yaw_ref = quat_to_rpy(ref_s.qx, ref_s.qy, ref_s.qz, ref_s.qw)
            roll_est, pitch_est, yaw_est = quat_to_rpy(s.qx, s.qy, s.qz, s.qw)
            if not all(
                math.isfinite(v) for v in (roll_ref, pitch_ref, yaw_ref, roll_est, pitch_est, yaw_est)
            ):
                continue

            err_roll_deg.append(wrap_to_pi(roll_est - roll_ref) * 180.0 / math.pi)
            err_pitch_deg.append(wrap_to_pi(pitch_est - pitch_ref) * 180.0 / math.pi)
            err_yaw_deg.append(wrap_to_pi(yaw_est - yaw_ref) * 180.0 / math.pi)

            ang = quat_angle_error_deg(s, ref_s)
            if ang is not None and math.isfinite(ang):
                err_angle_deg.append(ang)

        if err_yaw_deg:
            out["attitude_matched_samples"] = float(len(err_yaw_deg))
            out["roll_rmse_deg"] = rmse(err_roll_deg)
            out["pitch_rmse_deg"] = rmse(err_pitch_deg)
            out["yaw_rmse_deg"] = rmse(err_yaw_deg)
            out["roll_bias_deg"] = statistics.mean(err_roll_deg)
            out["pitch_bias_deg"] = statistics.mean(err_pitch_deg)
            out["yaw_bias_deg"] = statistics.mean(err_yaw_deg)
            out["yaw_mae_deg"] = statistics.mean(abs(v) for v in err_yaw_deg)
            if err_angle_deg:
                out["attitude_angle_rmse_deg"] = rmse(err_angle_deg)
                out["attitude_angle_mean_deg"] = statistics.mean(err_angle_deg)
        return out

    # yaw_reference == "gt_course": course from GT positions
    err_yaw_deg: List[float] = []
    for s in est:
        seg = match_segment_at_time(gt, gt_times, s.t_sec)
        if seg is None:
            continue
        left, right, gap_left, gap_right = seg
        if min(gap_left, gap_right) > max_time_gap_sec:
            continue
        dt = right.t_sec - left.t_sec
        if dt <= 0.0:
            continue
        dx = right.x - left.x
        dy = right.y - left.y
        speed_xy = math.hypot(dx, dy) / dt
        if attitude_min_speed_mps > 0.0 and speed_xy < attitude_min_speed_mps:
            continue
        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            continue
        yaw_ref = math.atan2(dy, dx)
        _, _, yaw_est = quat_to_rpy(s.qx, s.qy, s.qz, s.qw)
        if not (math.isfinite(yaw_ref) and math.isfinite(yaw_est)):
            continue
        err_yaw_deg.append(wrap_to_pi(yaw_est - yaw_ref) * 180.0 / math.pi)

    if err_yaw_deg:
        out["attitude_matched_samples"] = float(len(err_yaw_deg))
        out["yaw_rmse_deg"] = rmse(err_yaw_deg)
        out["yaw_bias_deg"] = statistics.mean(err_yaw_deg)
        out["yaw_mae_deg"] = statistics.mean(abs(v) for v in err_yaw_deg)
    return out


def percentile(sorted_values: Sequence[float], q: float) -> float:
    if not sorted_values:
        return float("nan")
    if q <= 0.0:
        return sorted_values[0]
    if q >= 1.0:
        return sorted_values[-1]

    p = q * (len(sorted_values) - 1)
    low = int(math.floor(p))
    high = int(math.ceil(p))
    if low == high:
        return sorted_values[low]
    ratio = p - low
    return (1.0 - ratio) * sorted_values[low] + ratio * sorted_values[high]


def evaluate(
    est_samples: Sequence[PoseSample],
    gt_samples: Sequence[PoseSample],
    max_time_gap_sec: float,
) -> Dict[str, float]:
    if max_time_gap_sec <= 0.0 or not math.isfinite(max_time_gap_sec):
        raise ValueError(f"max_time_gap_sec must be finite and > 0.0, got {max_time_gap_sec}")

    sq_err_xyz = 0.0
    sq_err_xy = 0.0
    err_norms: List[float] = []
    err_x: List[float] = []
    err_y: List[float] = []
    err_z: List[float] = []

    gt_times = [s.t_sec for s in gt_samples]
    for est in est_samples:
        matched = match_pose_at_time(gt_samples, gt_times, est.t_sec)
        if matched is None:
            continue
        gt_interp, gap_left, gap_right = matched

        # Use nearest neighbor gap gate so lower-rate ground truth can still be
        # evaluated with interpolation.
        if min(gap_left, gap_right) > max_time_gap_sec:
            continue

        dx = est.x - gt_interp.x
        dy = est.y - gt_interp.y
        dz = est.z - gt_interp.z
        e2_xy = dx * dx + dy * dy
        e2_xyz = e2_xy + dz * dz

        sq_err_xy += e2_xy
        sq_err_xyz += e2_xyz
        err_norms.append(math.sqrt(e2_xyz))
        err_x.append(dx)
        err_y.append(dy)
        err_z.append(dz)

    n = len(err_norms)
    if n == 0:
        raise RuntimeError(
            "No matched samples. Check topic time alignment, columns, and max_time_gap_sec."
        )

    err_norms_sorted = sorted(err_norms)
    bias_x = statistics.mean(err_x)
    bias_y = statistics.mean(err_y)
    bias_z = statistics.mean(err_z)

    # De-biased RMSE is useful when the two trajectories are expressed in frames
    # that can have a constant offset (e.g. independent ENU origins, antenna/IMU
    # lever arm). We still report the raw bias separately.
    sq_err_xy_nobias = 0.0
    sq_err_xyz_nobias = 0.0
    for dx, dy, dz in zip(err_x, err_y, err_z):
        ddx = dx - bias_x
        ddy = dy - bias_y
        ddz = dz - bias_z
        e2_xy = ddx * ddx + ddy * ddy
        sq_err_xy_nobias += e2_xy
        sq_err_xyz_nobias += e2_xy + ddz * ddz

    metrics: Dict[str, float] = {
        "matched_samples": float(n),
        "rmse_3d_m": math.sqrt(sq_err_xyz / n),
        "rmse_xy_m": math.sqrt(sq_err_xy / n),
        "rmse_3d_nobias_m": math.sqrt(sq_err_xyz_nobias / n),
        "rmse_xy_nobias_m": math.sqrt(sq_err_xy_nobias / n),
        "mean_3d_m": statistics.mean(err_norms),
        "median_3d_m": statistics.median(err_norms),
        "p95_3d_m": percentile(err_norms_sorted, 0.95),
        "max_3d_m": max(err_norms),
        "bias_x_m": bias_x,
        "bias_y_m": bias_y,
        "bias_z_m": bias_z,
    }
    return metrics


def format_metrics(metrics: Dict[str, float]) -> str:
    lines = [
        f"matched_samples: {int(metrics['matched_samples'])}",
        f"rmse_3d_m: {metrics['rmse_3d_m']:.6f}",
        f"rmse_xy_m: {metrics['rmse_xy_m']:.6f}",
        f"rmse_3d_nobias_m: {metrics['rmse_3d_nobias_m']:.6f}",
        f"rmse_xy_nobias_m: {metrics['rmse_xy_nobias_m']:.6f}",
        f"mean_3d_m: {metrics['mean_3d_m']:.6f}",
        f"median_3d_m: {metrics['median_3d_m']:.6f}",
        f"p95_3d_m: {metrics['p95_3d_m']:.6f}",
        f"max_3d_m: {metrics['max_3d_m']:.6f}",
        f"bias_x_m: {metrics['bias_x_m']:.6f}",
        f"bias_y_m: {metrics['bias_y_m']:.6f}",
        f"bias_z_m: {metrics['bias_z_m']:.6f}",
    ]
    if "attitude_matched_samples" in metrics:
        lines.append(f"attitude_matched_samples: {int(metrics['attitude_matched_samples'])}")
    if "yaw_rmse_deg" in metrics:
        lines.append(f"yaw_rmse_deg: {metrics['yaw_rmse_deg']:.3f}")
    if "yaw_bias_deg" in metrics:
        lines.append(f"yaw_bias_deg: {metrics['yaw_bias_deg']:.3f}")
    if "yaw_mae_deg" in metrics:
        lines.append(f"yaw_mae_deg: {metrics['yaw_mae_deg']:.3f}")
    if "roll_rmse_deg" in metrics:
        lines.append(f"roll_rmse_deg: {metrics['roll_rmse_deg']:.3f}")
    if "pitch_rmse_deg" in metrics:
        lines.append(f"pitch_rmse_deg: {metrics['pitch_rmse_deg']:.3f}")
    if "attitude_angle_rmse_deg" in metrics:
        lines.append(f"attitude_angle_rmse_deg: {metrics['attitude_angle_rmse_deg']:.3f}")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--estimated-csv", required=True, type=Path)
    p.add_argument("--ground-truth-csv", required=True, type=Path)
    p.add_argument(
        "--attitude-reference-csv",
        type=Path,
        default=None,
        help=(
            "optional orientation reference CSV (tools/record_pose_csv.py format). "
            "When provided, attitude metrics are computed in addition to trajectory metrics."
        ),
    )

    p.add_argument("--est-time-col", default="t_sec")
    p.add_argument("--est-x-col", default="x")
    p.add_argument("--est-y-col", default="y")
    p.add_argument("--est-z-col", default="z")
    p.add_argument("--est-time-scale", default=1.0, type=float)

    p.add_argument("--gt-time-col", default="t_sec")
    p.add_argument("--gt-x-col", default="x")
    p.add_argument("--gt-y-col", default="y")
    p.add_argument("--gt-z-col", default="z")
    p.add_argument("--gt-time-scale", default=1.0, type=float)

    p.add_argument("--max-time-gap-sec", default=0.1, type=float)
    p.add_argument(
        "--keep-zero-stamp",
        action="store_true",
        help="keep samples whose t_sec is exactly 0.0 (default: drop them)",
    )
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
        default="absolute",
        help=(
            "time alignment mode before matching. "
            "'relative' subtracts each CSV's first timestamp. "
            "'auto' selects absolute/relative based on which yields more matches."
        ),
    )
    p.add_argument(
        "--yaw-reference",
        choices=["auto", "gt_quat", "gt_course", "attitude_csv"],
        default="auto",
        help="reference yaw source for attitude metrics. auto prefers gt_quat (if available), then attitude_csv, otherwise gt_course",
    )
    p.add_argument(
        "--attitude-min-speed-mps",
        type=float,
        default=0.0,
        help=(
            "when > 0, compute attitude metrics only when ground-truth horizontal speed >= this threshold. "
            "Useful since yaw can be unobservable at standstill."
        ),
    )
    p.add_argument("--output-json", type=Path, default=None)
    return p


def main() -> int:
    args = build_parser().parse_args()

    try:
        est_samples = read_pose_csv(
            csv_path=args.estimated_csv,
            time_col=args.est_time_col,
            x_col=args.est_x_col,
            y_col=args.est_y_col,
            z_col=args.est_z_col,
            time_scale=args.est_time_scale,
        )
        gt_samples = read_pose_csv(
            csv_path=args.ground_truth_csv,
            time_col=args.gt_time_col,
            x_col=args.gt_x_col,
            y_col=args.gt_y_col,
            z_col=args.gt_z_col,
            time_scale=args.gt_time_scale,
        )
        attitude_ref: Optional[List[PoseSample]] = None
        if args.attitude_reference_csv is not None:
            attitude_ref = read_pose_csv(
                csv_path=args.attitude_reference_csv,
                time_col=args.est_time_col,
                x_col=args.est_x_col,
                y_col=args.est_y_col,
                z_col=args.est_z_col,
                time_scale=args.est_time_scale,
            )
        est_samples = filter_pose_samples(
            est_samples, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
        )
        gt_samples = filter_pose_samples(
            gt_samples, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
        )
        if attitude_ref is not None:
            attitude_ref = filter_pose_samples(
                attitude_ref, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
            )

        time_align_candidates = (
            ["absolute", "relative"] if args.time_align == "auto" else [args.time_align]
        )
        time_normalize_candidates = (
            ["none", "unix_to_gps_tow"]
            if args.time_normalize == "auto"
            else [args.time_normalize]
        )

        # Keep all successful candidates and select the best at the end.
        # Auto modes are only heuristics; prefer stable/low-error alignment over
        # maximizing matched_samples blindly (which can hide timestamp-domain mixups).
        candidates: List[Tuple[Dict[str, float], str, str, float, float]] = []
        used_time_align = "absolute"
        used_time_normalize = "none"
        est_time_offset_sec = 0.0
        gt_time_offset_sec = 0.0
        last_error: Optional[Exception] = None

        for time_normalize in time_normalize_candidates:
            est_norm = apply_time_normalize(
                est_samples, mode=time_normalize, gps_leap_seconds=args.gps_leap_seconds
            )
            gt_norm = apply_time_normalize(
                gt_samples, mode=time_normalize, gps_leap_seconds=args.gps_leap_seconds
            )
            for time_align in time_align_candidates:
                est_aligned = est_norm
                gt_aligned = gt_norm
                est_off = 0.0
                gt_off = 0.0
                if time_align == "relative":
                    if not est_norm or not gt_norm:
                        continue
                    est_off = est_norm[0].t_sec
                    gt_off = gt_norm[0].t_sec
                    est_aligned = shift_time(est_norm, est_off)
                    gt_aligned = shift_time(gt_norm, gt_off)

                try:
                    metrics = evaluate(
                        est_samples=est_aligned,
                        gt_samples=gt_aligned,
                        max_time_gap_sec=args.max_time_gap_sec,
                    )
                except Exception as e:  # pylint: disable=broad-except
                    last_error = e
                    continue
                candidates.append((metrics, time_align, time_normalize, est_off, gt_off))

        if not candidates:
            if last_error is not None:
                raise last_error
            raise RuntimeError(
                "No matched samples. Check topic time alignment, columns, and max_time_gap_sec."
            )

        # Avoid selecting alignments with too few matches, which can spuriously lower RMSE.
        max_matches = max(int(m["matched_samples"]) for m, *_ in candidates)
        min_matches = max(30, int(math.floor(max_matches * 0.05)))
        filtered = [c for c in candidates if int(c[0]["matched_samples"]) >= min_matches]
        if not filtered:
            filtered = candidates

        def score(c: Tuple[Dict[str, float], str, str, float, float]) -> Tuple[float, int, int]:
            metrics, time_align, time_normalize, _, _ = c
            rmse = float(metrics["rmse_3d_m"])
            matched = int(metrics["matched_samples"])
            transform_penalty = 0
            if time_align == "relative":
                transform_penalty += 1
            if time_normalize == "unix_to_gps_tow":
                transform_penalty += 1
            # Primary: lower RMSE. Secondary: more matches. Tertiary: fewer transforms.
            return rmse, -matched, transform_penalty

        best_metrics, used_time_align, used_time_normalize, est_time_offset_sec, gt_time_offset_sec = min(
            filtered, key=score
        )
        metrics = best_metrics

        # Optional attitude metrics using the selected alignment.
        est_norm = apply_time_normalize(
            est_samples, mode=used_time_normalize, gps_leap_seconds=args.gps_leap_seconds
        )
        gt_norm = apply_time_normalize(
            gt_samples, mode=used_time_normalize, gps_leap_seconds=args.gps_leap_seconds
        )
        att_norm = (
            apply_time_normalize(attitude_ref, mode=used_time_normalize, gps_leap_seconds=args.gps_leap_seconds)
            if attitude_ref is not None
            else None
        )
        est_aligned = shift_time(est_norm, est_time_offset_sec) if used_time_align == "relative" else est_norm
        gt_aligned = shift_time(gt_norm, gt_time_offset_sec) if used_time_align == "relative" else gt_norm
        att_aligned: Optional[List[PoseSample]] = None
        if att_norm is not None:
            if used_time_align == "relative":
                att_aligned = shift_time(att_norm, att_norm[0].t_sec) if att_norm else []
            else:
                att_aligned = list(att_norm)

        has_att_quat = bool(att_aligned) and not quat_sequence_is_identity(att_aligned or [])
        has_gt_quat = not quat_sequence_is_identity(gt_aligned)
        yaw_reference = args.yaw_reference
        if yaw_reference == "auto":
            # Prefer GT quaternion when available (true attitude), otherwise fall back to
            # recorded attitude reference, otherwise use GT course.
            if has_gt_quat:
                yaw_reference = "gt_quat"
            elif has_att_quat:
                yaw_reference = "attitude_csv"
            else:
                yaw_reference = "gt_course"
        if yaw_reference == "attitude_csv" and not has_att_quat:
            yaw_reference = "gt_quat" if has_gt_quat else "gt_course"
        if yaw_reference == "gt_quat" and not has_gt_quat:
            yaw_reference = "attitude_csv" if has_att_quat else "gt_course"

        att_metrics = compute_attitude_metrics(
            est=est_aligned,
            gt=gt_aligned,
            attitude_ref=att_aligned,
            max_time_gap_sec=args.max_time_gap_sec,
            yaw_reference=yaw_reference,
            attitude_min_speed_mps=args.attitude_min_speed_mps,
        )
        metrics.update(att_metrics)
    except Exception as e:  # pylint: disable=broad-except
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    print("Trajectory Evaluation")
    print(f"estimated_csv: {args.estimated_csv}")
    print(f"ground_truth_csv: {args.ground_truth_csv}")
    print(f"time_normalize: {used_time_normalize}")
    print(f"time_align: {used_time_align}")
    if "yaw_rmse_deg" in metrics:
        print(f"yaw_reference: {yaw_reference}")
    print(format_metrics(metrics))

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "estimated_csv": str(args.estimated_csv),
            "ground_truth_csv": str(args.ground_truth_csv),
            "attitude_reference_csv": str(args.attitude_reference_csv) if args.attitude_reference_csv else "",
            "max_time_gap_sec": args.max_time_gap_sec,
            "time_normalize": used_time_normalize,
            "gps_leap_seconds": args.gps_leap_seconds,
            "time_align": used_time_align,
            "est_time_offset_sec": est_time_offset_sec,
            "gt_time_offset_sec": gt_time_offset_sec,
            "yaw_reference": yaw_reference,
            "metrics": metrics,
        }
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        print(f"metrics_json: {args.output_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
