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
        out.append(PoseSample(t_sec=t_sec, x=s.x, y=s.y, z=s.z))
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
        PoseSample(t_sec=s.t_sec - offset_sec, x=s.x, y=s.y, z=s.z)
        for s in samples
    ]


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

        for line_no, row in enumerate(reader, start=2):
            try:
                t_raw = float(row[time_col])
                x = float(row[x_col])
                y = float(row[y_col])
                z = float(row[z_col])
            except (TypeError, ValueError) as e:
                raise ValueError(
                    f"{csv_path}:{line_no} contains non-numeric values for "
                    f"columns {time_col},{x_col},{y_col},{z_col}"
                ) from e

            t_sec = t_raw * time_scale
            if not all(math.isfinite(v) for v in (t_sec, x, y, z)):
                continue
            rows.append(PoseSample(t_sec=t_sec, x=x, y=y, z=z))

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
    metrics: Dict[str, float] = {
        "matched_samples": float(n),
        "rmse_3d_m": math.sqrt(sq_err_xyz / n),
        "rmse_xy_m": math.sqrt(sq_err_xy / n),
        "mean_3d_m": statistics.mean(err_norms),
        "median_3d_m": statistics.median(err_norms),
        "p95_3d_m": percentile(err_norms_sorted, 0.95),
        "max_3d_m": max(err_norms),
        "bias_x_m": statistics.mean(err_x),
        "bias_y_m": statistics.mean(err_y),
        "bias_z_m": statistics.mean(err_z),
    }
    return metrics


def format_metrics(metrics: Dict[str, float]) -> str:
    lines = [
        f"matched_samples: {int(metrics['matched_samples'])}",
        f"rmse_3d_m: {metrics['rmse_3d_m']:.6f}",
        f"rmse_xy_m: {metrics['rmse_xy_m']:.6f}",
        f"mean_3d_m: {metrics['mean_3d_m']:.6f}",
        f"median_3d_m: {metrics['median_3d_m']:.6f}",
        f"p95_3d_m: {metrics['p95_3d_m']:.6f}",
        f"max_3d_m: {metrics['max_3d_m']:.6f}",
        f"bias_x_m: {metrics['bias_x_m']:.6f}",
        f"bias_y_m: {metrics['bias_y_m']:.6f}",
        f"bias_z_m: {metrics['bias_z_m']:.6f}",
    ]
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--estimated-csv", required=True, type=Path)
    p.add_argument("--ground-truth-csv", required=True, type=Path)

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
        est_samples = filter_pose_samples(
            est_samples, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
        )
        gt_samples = filter_pose_samples(
            gt_samples, drop_stamp_zero=not args.keep_zero_stamp, stamp_zero_abs_tol=1e-12
        )

        time_align_candidates = (
            ["absolute", "relative"] if args.time_align == "auto" else [args.time_align]
        )
        time_normalize_candidates = (
            ["none", "unix_to_gps_tow"]
            if args.time_normalize == "auto"
            else [args.time_normalize]
        )

        best_metrics: Optional[Dict[str, float]] = None
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

                if best_metrics is None:
                    best_metrics = metrics
                    used_time_align = time_align
                    used_time_normalize = time_normalize
                    est_time_offset_sec = est_off
                    gt_time_offset_sec = gt_off
                    continue

                matched = int(metrics["matched_samples"])
                best_matched = int(best_metrics["matched_samples"])
                rmse = float(metrics["rmse_3d_m"])
                best_rmse = float(best_metrics["rmse_3d_m"])

                better = False
                if matched > best_matched:
                    better = True
                elif matched == best_matched:
                    if rmse < best_rmse - 1e-12:
                        better = True
                    elif abs(rmse - best_rmse) <= 1e-12:
                        # Prefer smaller transformations when equivalent.
                        if used_time_align == "relative" and time_align == "absolute":
                            better = True
                        elif used_time_align == time_align:
                            if used_time_normalize == "unix_to_gps_tow" and time_normalize == "none":
                                better = True

                if better:
                    best_metrics = metrics
                    used_time_align = time_align
                    used_time_normalize = time_normalize
                    est_time_offset_sec = est_off
                    gt_time_offset_sec = gt_off

        if best_metrics is None:
            if last_error is not None:
                raise last_error
            raise RuntimeError(
                "No matched samples. Check topic time alignment, columns, and max_time_gap_sec."
            )

        metrics = best_metrics
    except Exception as e:  # pylint: disable=broad-except
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    print("Trajectory Evaluation")
    print(f"estimated_csv: {args.estimated_csv}")
    print(f"ground_truth_csv: {args.ground_truth_csv}")
    print(f"time_normalize: {used_time_normalize}")
    print(f"time_align: {used_time_align}")
    print(format_metrics(metrics))

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "estimated_csv": str(args.estimated_csv),
            "ground_truth_csv": str(args.ground_truth_csv),
            "max_time_gap_sec": args.max_time_gap_sec,
            "time_normalize": used_time_normalize,
            "gps_leap_seconds": args.gps_leap_seconds,
            "time_align": used_time_align,
            "est_time_offset_sec": est_time_offset_sec,
            "gt_time_offset_sec": gt_time_offset_sec,
            "metrics": metrics,
        }
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        print(f"metrics_json: {args.output_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
