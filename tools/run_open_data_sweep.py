#!/usr/bin/env python3
"""Run EKF parameter sweep on a ROS2 bag and evaluate trajectory error metrics."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
RECORD_SCRIPT = SCRIPT_DIR / "record_pose_csv.py"
EVAL_SCRIPT = SCRIPT_DIR / "evaluate_trajectory.py"
NAVSATFIX_SCRIPT = SCRIPT_DIR / "navsatfix_to_pose.py"
PLOT_SCRIPT = SCRIPT_DIR / "plot_pose_csv.py"
APPLANIX_SCRIPT = SCRIPT_DIR / "applanix_nav_solution_to_pose.py"


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
        "--applanix-origin-navsatfix-topic",
        default=None,
        help=(
            "optional NavSatFix topic used by tools/applanix_nav_solution_to_pose.py to set a shared ENU origin. "
            "Useful for INS ground truth evaluation to reduce frame offset vs GNSS-origin estimates."
        ),
    )
    p.add_argument("--applanix-origin-navsatfix-qos-depth", type=int, default=10)

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

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "summary.csv"
    ranking_path = args.output_dir / "ranking_by_rmse_3d.csv"
    ranking_nobias_path = args.output_dir / "ranking_by_rmse_3d_nobias.csv"

    print(f"bag_path: {args.bag_path}")
    print(f"param_grid_json: {args.param_grid_json}")
    print(f"total_runs: {len(combinations)}")
    print(f"output_dir: {args.output_dir}")

    rows: List[Dict[str, object]] = []
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

            if args.publish_initial_pose:
                x, y, z, qx, qy, qz, qw = initial_pose
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
            with (run_dir / "bag_play.log").open("w", encoding="utf-8") as f:
                subprocess.run(play_cmd, check=True, stdout=f, stderr=subprocess.STDOUT)

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

    successful = [
        r for r in rows if r["status"] == "ok" and isinstance(r.get("rmse_3d_m"), (int, float))
    ]
    successful_sorted = sorted(successful, key=lambda r: float(r["rmse_3d_m"]))
    if successful_sorted:
        fieldnames = [
            "run_id",
            "status",
            *keys,
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
        successful_nobias_sorted = sorted(successful_nobias, key=lambda r: float(r["rmse_3d_nobias_m"]))
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
                ]
                if (run_dir / "attitude_reference.csv").exists():
                    plot_cmd.extend(["--attitude-reference-csv", str(run_dir / "attitude_reference.csv")])

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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
