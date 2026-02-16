#!/usr/bin/env python3
"""Run open-data sweep on multiple Autoware Istanbul all-sensors bags and summarize results.

This is a thin wrapper around tools/run_open_data_sweep.py for convenience.

Typical usage (after downloading bags under data/istanbul/):

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ROS_LOG_DIR=/tmp/ros2_logs \\
    python3 src/kalman_filter_localization/tools/run_istanbul_suite.py \\
      --output-dir /tmp/kfl_istanbul_suite
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
SWEEP_SCRIPT = SCRIPT_DIR / "run_open_data_sweep.py"


def default_ws_root() -> Path:
    # Expect: <ws>/src/kalman_filter_localization/tools/run_istanbul_suite.py
    # If that layout is not present, fall back to CWD.
    for parent in SCRIPT_DIR.parents:
        if (parent / "src" / "kalman_filter_localization").exists():
            return parent
    return Path.cwd()


def read_best_row(ranking_csv: Path) -> Optional[Dict[str, str]]:
    if not ranking_csv.exists():
        return None
    with ranking_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            return row
    return None


def copy_best_plots(best_row: Dict[str, str], bag_name: str, bag_out_dir: Path, suite_out_dir: Path) -> None:
    run_id = best_row.get("run_id", "")
    if not run_id:
        return
    run_dir = bag_out_dir / run_id
    xy = run_dir / f"{run_id}_trajectory_xy.png"
    ts = run_dir / f"{run_id}_timeseries_z_rpy.png"
    if not (xy.exists() or ts.exists()):
        return

    dst_dir = suite_out_dir / "best_plots"
    dst_dir.mkdir(parents=True, exist_ok=True)
    if xy.exists():
        shutil.copyfile(xy, dst_dir / f"{bag_name}_trajectory_xy.png")
    if ts.exists():
        shutil.copyfile(ts, dst_dir / f"{bag_name}_timeseries_z_rpy.png")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ws-root", type=Path, default=default_ws_root(), help="ROS2 workspace root")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument(
        "--param-grid-json",
        type=Path,
        default=SCRIPT_DIR / "param_grid_istanbul_quick.json",
    )
    p.add_argument("--play-rate", type=float, default=20.0)
    p.add_argument("--max-runs", type=int, default=0, help="0 means all combinations")
    p.add_argument(
        "--ground-truth",
        choices=["gnss_pose", "ins_pose"],
        default="gnss_pose",
        help="ground truth topic to evaluate against (default: gnss_pose)",
    )
    p.add_argument(
        "--bags",
        nargs="*",
        default=[f"all-sensors-bag{i}_compressed" for i in range(1, 7)],
        help="bag directory names under data/istanbul/",
    )
    p.add_argument("--data-dir", type=Path, default=Path("data/istanbul"))
    p.add_argument("--estimated-qos-depth", type=int, default=10)
    p.add_argument("--ground-truth-qos-depth", type=int, default=10)
    p.add_argument(
        "--ekf-output-stamp-source",
        choices=["latest_input", "imu", "ros_time"],
        default="latest_input",
        help="forwarded to tools/run_open_data_sweep.py",
    )
    p.add_argument(
        "--eval-max-time-gap-sec",
        type=float,
        default=0.1,
        help="forwarded to tools/run_open_data_sweep.py",
    )
    p.add_argument(
        "--no-attitude-reference",
        action="store_true",
        help="do not record IMU attitude reference CSV (plots will use GT quaternion if available)",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    ws_root = args.ws_root.resolve()
    data_dir = (ws_root / args.data_dir).resolve()
    suite_out = args.output_dir.resolve()
    suite_out.mkdir(parents=True, exist_ok=True)

    if not SWEEP_SCRIPT.exists():
        print(f"ERROR: missing: {SWEEP_SCRIPT}", file=sys.stderr)
        return 2
    if not (ws_root / "src" / "kalman_filter_localization").exists():
        print(
            f"ERROR: --ws-root does not look like a ROS2 ws root: {ws_root}",
            file=sys.stderr,
        )
        return 2
    if not data_dir.exists():
        print(f"ERROR: data dir not found: {data_dir}", file=sys.stderr)
        return 2
    if not args.param_grid_json.exists():
        print(f"ERROR: param grid not found: {args.param_grid_json}", file=sys.stderr)
        return 2

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    summary_rows: List[Dict[str, str]] = []

    for bag_name in args.bags:
        bag_path = data_dir / bag_name
        if not bag_path.exists():
            print(f"skip: bag not found: {bag_path}")
            continue

        bag_out_dir = suite_out / f"{bag_name}_{stamp}"
        play_topics = ["/sensing/imu/imu_data", "/gnss/fix"]
        ground_truth_topic = "/gnss_pose" if args.ground_truth == "gnss_pose" else "/ins_pose"
        if args.ground_truth == "ins_pose":
            play_topics.append("/lvx_client/gsof/ins_solution_49")
        cmd = [
            sys.executable,
            str(SWEEP_SCRIPT),
            "--bag-path",
            str(bag_path),
            "--param-grid-json",
            str(args.param_grid_json),
            "--output-dir",
            str(bag_out_dir),
            "--imu-topic",
            "/sensing/imu/imu_data",
            "--gnss-topic",
            "/gnss_pose",
            "--ground-truth-topic",
            ground_truth_topic,
            "--estimated-qos-depth",
            str(args.estimated_qos_depth),
            "--ground-truth-qos-depth",
            str(args.ground_truth_qos_depth),
            "--play-topics",
            *play_topics,
            "--enable-navsatfix-to-pose",
            "--navsatfix-input-topic",
            "/gnss/fix",
            "--navsatfix-output-topic",
            "/gnss_pose",
            "--ekf-output-stamp-source",
            args.ekf_output_stamp_source,
            "--plot-best",
            "--eval-max-time-gap-sec",
            f"{args.eval_max_time_gap_sec:.12g}",
            "--play-rate",
            f"{args.play_rate:.12g}",
        ]
        if args.ground_truth == "ins_pose":
            # Applanix INS reference in Istanbul bags:
            # /lvx_client/gsof/ins_solution_49 (applanix_msgs/msg/NavigationSolutionGsof49)
            cmd += [
                "--enable-applanix-to-pose",
                "--applanix-input-topic",
                "/lvx_client/gsof/ins_solution_49",
                "--applanix-output-topic",
                "/ins_pose",
                # Align INS pose origin to GNSS origin to reduce a constant frame offset.
                "--applanix-origin-navsatfix-topic",
                "/gnss/fix",
            ]

        if not args.no_attitude_reference:
            cmd += [
                "--attitude-reference-topic",
                "/sensing/imu/imu_data",
                "--attitude-reference-msg-type",
                "imu",
            ]
        if args.max_runs > 0:
            cmd += ["--max-runs", str(args.max_runs)]

        print(f"\n=== {bag_name} ===")
        proc = subprocess.run(cmd, check=False)

        best = read_best_row(bag_out_dir / "ranking_by_rmse_3d.csv")
        row: Dict[str, str] = {
            "bag": bag_name,
            "bag_path": str(bag_path),
            "output_dir": str(bag_out_dir),
            "exit_code": str(proc.returncode),
        }
        if best:
            row.update(
                {
                    "best_run_id": best.get("run_id", ""),
                    "rmse_3d_m": best.get("rmse_3d_m", ""),
                    "rmse_xy_m": best.get("rmse_xy_m", ""),
                    "bias_z_m": best.get("bias_z_m", ""),
                    "matched_samples": best.get("matched_samples", ""),
                }
            )
            copy_best_plots(best, bag_name, bag_out_dir, suite_out)
        summary_rows.append(row)

    summary_csv = suite_out / f"istanbul_suite_summary_{stamp}.csv"
    fieldnames = [
        "bag",
        "bag_path",
        "output_dir",
        "exit_code",
        "best_run_id",
        "matched_samples",
        "rmse_3d_m",
        "rmse_xy_m",
        "bias_z_m",
    ]
    with summary_csv.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in summary_rows:
            w.writerow(r)

    print(f"\nsummary_csv: {summary_csv}")
    best_plots = suite_out / "best_plots"
    if best_plots.exists():
        print(f"best_plots_dir: {best_plots}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
