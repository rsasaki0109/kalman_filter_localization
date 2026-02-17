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
import html
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
SWEEP_SCRIPT = SCRIPT_DIR / "run_open_data_sweep.py"
REPORT_NAME_PREFIX = "open_data_suite_report"


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


def copy_best_plots(
    best_row: Dict[str, str],
    bag_name: str,
    bag_out_dir: Path,
    suite_out_dir: Path,
    *,
    suffix: str = "",
) -> None:
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
        shutil.copyfile(xy, dst_dir / f"{bag_name}{suffix}_trajectory_xy.png")
    if ts.exists():
        shutil.copyfile(ts, dst_dir / f"{bag_name}{suffix}_timeseries_z_rpy.png")


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
        "--attitude-min-speed-mps",
        type=float,
        default=0.5,
        help="forwarded to tools/run_open_data_sweep.py",
    )
    p.add_argument(
        "--initial-yaw-source-topic",
        default=None,
        help=(
            "topic used to read yaw for initial pose (forwarded to run_open_data_sweep.py). "
            "If omitted and --ground-truth is ins_pose, defaults to the ground-truth topic."
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
        default=5.0,
        help="timeout for initial yaw sample read in seconds",
    )
    p.add_argument(
        "--no-attitude-reference",
        action="store_true",
        help="do not record IMU attitude reference CSV (plots will use GT quaternion if available)",
    )
    p.add_argument(
        "--applanix-orientation-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help="forwarded to tools/run_open_data_sweep.py --applanix-orientation-mode (INS pose orientation conversion)",
    )
    p.add_argument(
        "--enable-applanix-to-imu",
        action="store_true",
        default=False,
        help=(
            "start tools/gsof49_to_imu.py during each run (to convert GSOF49 INS -> Imu). "
            "When --ground-truth is ins_pose, this is enabled automatically."
        ),
    )
    p.add_argument(
        "--applanix-imu-input-topic",
        default="/lvx_client/gsof/ins_solution_49",
        help="forwarded to tools/run_open_data_sweep.py --applanix-imu-input-topic",
    )
    p.add_argument(
        "--applanix-imu-output-topic",
        default="/ins_imu",
        help="forwarded to tools/run_open_data_sweep.py --applanix-imu-output-topic",
    )
    p.add_argument(
        "--applanix-imu-output-frame-id",
        default="base_link",
        help="forwarded to tools/run_open_data_sweep.py --applanix-imu-output-frame-id",
    )
    p.add_argument("--applanix-imu-qos-depth", type=int, default=10)
    p.add_argument(
        "--applanix-imu-output-mode",
        choices=["identity", "raw_rpy", "ros"],
        default="ros",
        help="forwarded to tools/run_open_data_sweep.py --applanix-imu-output-mode",
    )
    return p


def write_html_report(
    *,
    suite_out: Path,
    stamp: str,
    summary_csv: Path,
    summary_rows: List[Dict[str, str]],
) -> Path:
    report_path = suite_out / f"{REPORT_NAME_PREFIX}_{stamp}.html"

    def img_tag(rel_src: str, alt: str) -> str:
        src = html.escape(rel_src)
        alt_esc = html.escape(alt)
        return f'<img src="{src}" alt="{alt_esc}" loading="lazy" />'

    best_plots = suite_out / "best_plots"
    cards: List[str] = []
    for r in summary_rows:
        bag = r.get("bag", "")
        if not bag:
            continue
        bag_esc = html.escape(bag)

        def rel(p: Path) -> str:
            try:
                return str(p.relative_to(suite_out))
            except Exception:  # pylint: disable=broad-except
                return str(p)

        out_dir = Path(r.get("output_dir", ""))
        out_dir_rel = html.escape(rel(out_dir)) if out_dir.as_posix() else ""

        # Images are copied by copy_best_plots() if present.
        raw_xy = best_plots / f"{bag}_trajectory_xy.png"
        raw_ts = best_plots / f"{bag}_timeseries_z_rpy.png"
        nb_xy = best_plots / f"{bag}_nobias_trajectory_xy.png"
        nb_ts = best_plots / f"{bag}_nobias_timeseries_z_rpy.png"

        def maybe_img(p: Path, alt: str) -> str:
            return img_tag(rel(p), alt) if p.exists() else "<div class=\"missing\">(missing)</div>"

        cards.append(
            "\n".join(
                [
                    "<section class=\"bag\">",
                    f"<h2>{bag_esc}</h2>",
                    f"<div class=\"meta\"><div><b>output_dir</b>: <code>{out_dir_rel}</code></div></div>",
                    "<div class=\"grid\">",
                    "<div class=\"panel\">",
                    "<h3>Left: Best (rmse_3d_m)</h3>",
                    "<div class=\"kv\">",
                    f"<div><b>run_id</b>: <code>{html.escape(r.get('best_run_id',''))}</code></div>",
                    f"<div><b>matched</b>: <code>{html.escape(r.get('matched_samples',''))}</code></div>",
                    f"<div><b>rmse_3d_m</b>: <code>{html.escape(r.get('rmse_3d_m',''))}</code></div>",
                    f"<div><b>rmse_3d_nobias_m</b>: <code>{html.escape(r.get('rmse_3d_nobias_m',''))}</code></div>",
                    f"<div><b>yaw_ref</b>: <code>{html.escape(r.get('yaw_reference',''))}</code></div>",
                    f"<div><b>yaw_rmse_deg</b>: <code>{html.escape(r.get('yaw_rmse_deg',''))}</code></div>",
                    f"<div><b>att_angle_rmse_deg</b>: <code>{html.escape(r.get('attitude_angle_rmse_deg',''))}</code></div>",
                    "</div>",
                    "<div class=\"imgs\">",
                    maybe_img(raw_xy, f"{bag} trajectory XY (best)"),
                    maybe_img(raw_ts, f"{bag} timeseries z+RPY (best)"),
                    "</div>",
                    "</div>",
                    "<div class=\"panel\">",
                    "<h3>Right: Best (rmse_3d_nobias_m)</h3>",
                    "<div class=\"kv\">",
                    f"<div><b>run_id</b>: <code>{html.escape(r.get('best_nobias_run_id',''))}</code></div>",
                    f"<div><b>matched</b>: <code>{html.escape(r.get('best_nobias_matched_samples',''))}</code></div>",
                    f"<div><b>rmse_3d_m</b>: <code>{html.escape(r.get('best_nobias_rmse_3d_m',''))}</code></div>",
                    f"<div><b>rmse_3d_nobias_m</b>: <code>{html.escape(r.get('best_nobias_rmse_3d_nobias_m',''))}</code></div>",
                    f"<div><b>yaw_ref</b>: <code>{html.escape(r.get('best_nobias_yaw_reference',''))}</code></div>",
                    f"<div><b>yaw_rmse_deg</b>: <code>{html.escape(r.get('best_nobias_yaw_rmse_deg',''))}</code></div>",
                    f"<div><b>att_angle_rmse_deg</b>: <code>{html.escape(r.get('best_nobias_attitude_angle_rmse_deg',''))}</code></div>",
                    "</div>",
                    "<div class=\"imgs\">",
                    maybe_img(nb_xy, f"{bag} trajectory XY (best nobias)"),
                    maybe_img(nb_ts, f"{bag} timeseries z+RPY (best nobias)"),
                    "</div>",
                    "</div>",
                    "</div>",
                    "</section>",
                ]
            )
        )

    css = """
    :root { color-scheme: light; }
    body { font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Ubuntu, Cantarell, Noto Sans, sans-serif; margin: 24px; }
    h1 { margin: 0 0 8px 0; }
    .sub { color: #333; margin: 0 0 18px 0; }
    code { background: #f3f3f3; padding: 2px 6px; border-radius: 6px; }
    .bag { border-top: 1px solid #ddd; padding-top: 18px; margin-top: 18px; }
    .meta { margin: 8px 0 14px 0; color: #333; }
    .grid { display: grid; grid-template-columns: 1fr; gap: 16px; }
    @media (min-width: 1100px) { .grid { grid-template-columns: 1fr 1fr; } }
    .panel { border: 1px solid #ddd; border-radius: 12px; padding: 14px; background: #fff; }
    .kv { display: grid; grid-template-columns: 1fr; gap: 6px; margin-bottom: 10px; }
    .imgs { display: grid; grid-template-columns: 1fr; gap: 12px; }
    img { width: 100%; height: auto; border: 1px solid #eee; border-radius: 10px; }
    .missing { color: #888; font-style: italic; padding: 12px; border: 1px dashed #ccc; border-radius: 10px; }
    """

    summary_rel = html.escape(str(summary_csv.relative_to(suite_out)))
    doc = "\n".join(
        [
            "<!doctype html>",
            "<html>",
            "<head>",
            "<meta charset=\"utf-8\" />",
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />",
            f"<title>{REPORT_NAME_PREFIX}_{html.escape(stamp)}</title>",
            f"<style>{css}</style>",
            "</head>",
            "<body>",
            f"<h1>Open-Data Suite Report ({html.escape(stamp)})</h1>",
            f"<p class=\"sub\">summary_csv: <code>{summary_rel}</code></p>",
            "<p class=\"sub\"><b>Legend:</b> "
            "left panel = best by <code>rmse_3d_m</code> (absolute RMSE, bias included), "
            "right panel = best by <code>rmse_3d_nobias_m</code> (bias removed before RMSE)."
            "</p>",
            *cards,
            "</body>",
            "</html>",
        ]
    )
    report_path.write_text(doc, encoding="utf-8")
    return report_path


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
        initial_yaw_source_topic = args.initial_yaw_source_topic
        if initial_yaw_source_topic is None and args.ground_truth == "ins_pose":
            initial_yaw_source_topic = ground_truth_topic
        use_ins_imu_topic = bool(args.ground_truth == "ins_pose" or args.enable_applanix_to_imu)
        imu_topic = args.applanix_imu_output_topic if use_ins_imu_topic else "/sensing/imu/imu_data"
        attitude_ref_topic = imu_topic if use_ins_imu_topic else "/sensing/imu/imu_data"

        cmd = [
            sys.executable,
            str(SWEEP_SCRIPT),
            "--bag-path",
            str(bag_path),
            "--param-grid-json",
            str(args.param_grid_json),
            "--output-dir",
            str(bag_out_dir),
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
            "--attitude-min-speed-mps",
            f"{args.attitude_min_speed_mps:.12g}",
            "--play-rate",
            f"{args.play_rate:.12g}",
            "--imu-topic",
            imu_topic,
        ]
        if initial_yaw_source_topic:
            cmd += [
                "--initial-yaw-source-topic",
                initial_yaw_source_topic,
                "--initial-yaw-source-msg-type",
                args.initial_yaw_source_msg_type,
                "--initial-yaw-qos-depth",
                str(args.initial_yaw_qos_depth),
                "--initial-yaw-timeout-sec",
                f"{args.initial_yaw_timeout_sec:.12g}",
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
                "--applanix-orientation-mode",
                args.applanix_orientation_mode,
                # Align INS pose origin to GNSS origin to reduce a constant frame offset.
                "--applanix-origin-navsatfix-topic",
                "/gnss/fix",
            ]

        if args.ground_truth == "ins_pose" or args.enable_applanix_to_imu:
            cmd += [
                "--enable-applanix-to-imu",
                "--applanix-imu-input-topic",
                args.applanix_imu_input_topic,
                "--applanix-imu-output-topic",
                args.applanix_imu_output_topic,
                "--applanix-imu-output-frame-id",
                args.applanix_imu_output_frame_id,
                "--applanix-imu-qos-depth",
                str(args.applanix_imu_qos_depth),
                "--applanix-imu-output-mode",
                args.applanix_imu_output_mode,
            ]

        if not args.no_attitude_reference:
            cmd += [
                "--attitude-reference-topic",
                attitude_ref_topic,
                "--attitude-reference-msg-type",
                "imu",
            ]
        if args.max_runs > 0:
            cmd += ["--max-runs", str(args.max_runs)]

        print(f"\n=== {bag_name} ===")
        proc = subprocess.run(cmd, check=False)

        best = read_best_row(bag_out_dir / "ranking_by_rmse_3d.csv")
        best_nobias = read_best_row(bag_out_dir / "ranking_by_rmse_3d_nobias.csv")
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
                    "rmse_3d_nobias_m": best.get("rmse_3d_nobias_m", ""),
                    "rmse_xy_nobias_m": best.get("rmse_xy_nobias_m", ""),
                    "bias_z_m": best.get("bias_z_m", ""),
                    "matched_samples": best.get("matched_samples", ""),
                    "yaw_reference": best.get("yaw_reference", ""),
                    "yaw_rmse_deg": best.get("yaw_rmse_deg", ""),
                    "roll_rmse_deg": best.get("roll_rmse_deg", ""),
                    "pitch_rmse_deg": best.get("pitch_rmse_deg", ""),
                    "attitude_angle_rmse_deg": best.get("attitude_angle_rmse_deg", ""),
                }
            )
            copy_best_plots(best, bag_name, bag_out_dir, suite_out)
        if best_nobias:
            row.update(
                {
                    "best_nobias_run_id": best_nobias.get("run_id", ""),
                    "best_nobias_matched_samples": best_nobias.get("matched_samples", ""),
                    "best_nobias_rmse_3d_m": best_nobias.get("rmse_3d_m", ""),
                    "best_nobias_rmse_xy_m": best_nobias.get("rmse_xy_m", ""),
                    "best_nobias_rmse_3d_nobias_m": best_nobias.get("rmse_3d_nobias_m", ""),
                    "best_nobias_rmse_xy_nobias_m": best_nobias.get("rmse_xy_nobias_m", ""),
                    "best_nobias_bias_z_m": best_nobias.get("bias_z_m", ""),
                    "best_nobias_yaw_reference": best_nobias.get("yaw_reference", ""),
                    "best_nobias_yaw_rmse_deg": best_nobias.get("yaw_rmse_deg", ""),
                    "best_nobias_roll_rmse_deg": best_nobias.get("roll_rmse_deg", ""),
                    "best_nobias_pitch_rmse_deg": best_nobias.get("pitch_rmse_deg", ""),
                    "best_nobias_attitude_angle_rmse_deg": best_nobias.get(
                        "attitude_angle_rmse_deg", ""
                    ),
                }
            )
            copy_best_plots(best_nobias, bag_name, bag_out_dir, suite_out, suffix="_nobias")
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
        "rmse_3d_nobias_m",
        "rmse_xy_nobias_m",
        "bias_z_m",
        "yaw_reference",
        "yaw_rmse_deg",
        "roll_rmse_deg",
        "pitch_rmse_deg",
        "attitude_angle_rmse_deg",
        "best_nobias_run_id",
        "best_nobias_matched_samples",
        "best_nobias_rmse_3d_m",
        "best_nobias_rmse_xy_m",
        "best_nobias_rmse_3d_nobias_m",
        "best_nobias_rmse_xy_nobias_m",
        "best_nobias_bias_z_m",
        "best_nobias_yaw_reference",
        "best_nobias_yaw_rmse_deg",
        "best_nobias_roll_rmse_deg",
        "best_nobias_pitch_rmse_deg",
        "best_nobias_attitude_angle_rmse_deg",
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
    report_path = write_html_report(
        suite_out=suite_out, stamp=stamp, summary_csv=summary_csv, summary_rows=summary_rows
    )
    print(f"report_html: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
