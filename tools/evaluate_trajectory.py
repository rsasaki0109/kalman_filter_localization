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
    samples: Sequence[PoseSample], t_sec: float
) -> Optional[Tuple[PoseSample, float, float]]:
    if len(samples) == 0:
        return None
    if len(samples) == 1:
        if math.isclose(samples[0].t_sec, t_sec, rel_tol=0.0, abs_tol=1e-12):
            return samples[0], 0.0, 0.0
        return None

    times = [s.t_sec for s in samples]
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

    for est in est_samples:
        matched = match_pose_at_time(gt_samples, est.t_sec)
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
        metrics = evaluate(
            est_samples=est_samples,
            gt_samples=gt_samples,
            max_time_gap_sec=args.max_time_gap_sec,
        )
    except Exception as e:  # pylint: disable=broad-except
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    print("Trajectory Evaluation")
    print(f"estimated_csv: {args.estimated_csv}")
    print(f"ground_truth_csv: {args.ground_truth_csv}")
    print(format_metrics(metrics))

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "estimated_csv": str(args.estimated_csv),
            "ground_truth_csv": str(args.ground_truth_csv),
            "max_time_gap_sec": args.max_time_gap_sec,
            "metrics": metrics,
        }
        args.output_json.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
        print(f"metrics_json: {args.output_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
