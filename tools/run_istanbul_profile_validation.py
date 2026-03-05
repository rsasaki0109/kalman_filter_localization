#!/usr/bin/env python3
"""Run one selected Istanbul profile per bag and summarize the results."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
SWEEP_SCRIPT = SCRIPT_DIR / "run_open_data_sweep.py"

SHARED_PARAMS: Dict[str, object] = {
    "gravity_mps2": 0.0,
    "use_imu_orientation": True,
    "use_imu_orientation_covariance": True,
    "var_imu_orientation_rpy": 0.001,
    "var_imu_w": 0.02,
    "var_imu_acc": 0.05,
    "var_gnss_xy": 0.02,
    "var_gnss_z": 0.1,
    "max_imu_dt_sec": 0.5,
    "use_gnss_velocity": False,
}

BAG46_PARAMS: Dict[str, object] = {
    **SHARED_PARAMS,
    "var_imu_orientation_rpy": 0.0015,
    "use_gnss_velocity": True,
    "propagate_gnss_velocity_cross_state": True,
    "var_gnss_velocity_xy": 0.02,
    "var_gnss_velocity_z": 0.5,
    "min_gnss_velocity_distance_m": 0.05,
    "max_gnss_velocity_dt_sec": 1.0,
    "max_gnss_velocity_innovation_mps": 5.0,
}


def default_ws_root() -> Path:
    for parent in SCRIPT_DIR.parents:
        if (parent / "src" / "kalman_filter_localization").exists():
            return parent
    return Path.cwd()


def choose_profile(bag_name: str) -> Tuple[str, Dict[str, object]]:
    if (
        "all-sensors-bag4" in bag_name
        or "all-sensors-bag5" in bag_name
        or "all-sensors-bag6" in bag_name
    ):
        return "istanbul_all_sensors_bag4_6.yaml", dict(BAG46_PARAMS)
    return "istanbul_all_sensors_bag.yaml", dict(SHARED_PARAMS)


def write_single_grid(path: Path, params: Dict[str, object]) -> None:
    grid = {k: [v] for k, v in params.items()}
    path.write_text(json.dumps(grid, indent=2) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ws-root", type=Path, default=default_ws_root())
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--data-dir", type=Path, default=Path("data/istanbul"))
    p.add_argument(
        "--bags",
        nargs="*",
        default=[f"all-sensors-bag{i}_compressed" for i in range(1, 7)],
    )
    p.add_argument("--play-rate", type=float, default=1.0)
    p.add_argument(
        "--thresholds-json",
        type=Path,
        default=None,
        help="optional JSON file mapping bag name -> {max_rmse_3d_m: ...}",
    )
    return p


def load_thresholds(path: Optional[Path]) -> Dict[str, Dict[str, float]]:
    if path is None:
        return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path} must contain a JSON object")
    thresholds: Dict[str, Dict[str, float]] = {}
    for bag, cfg in raw.items():
        if not isinstance(bag, str) or not isinstance(cfg, dict):
            raise ValueError(f"{path} entries must be bag-name -> object")
        val = cfg.get("max_rmse_3d_m")
        if val is None:
            continue
        max_rmse_3d_m = float(val)
        if not math.isfinite(max_rmse_3d_m) or max_rmse_3d_m <= 0.0:
            raise ValueError(f"invalid max_rmse_3d_m for {bag}: {val}")
        thresholds[bag] = {"max_rmse_3d_m": max_rmse_3d_m}
    return thresholds


def read_first_row(csv_path: Path) -> Dict[str, str]:
    if not csv_path.exists():
        return {}
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        return next(csv.DictReader(f), {}) or {}


def write_html_report(output_dir: Path, rows: List[Dict[str, str]]) -> Path:
    report_path = output_dir / "kf_profile_validation_report.html"

    def esc(value: object) -> str:
        return html.escape("" if value is None else str(value))

    def rel(path: Path) -> str:
        try:
            return str(path.relative_to(output_dir))
        except ValueError:
            return str(path)

    body: List[str] = [
        "<!doctype html>",
        "<html>",
        "<head>",
        '<meta charset="utf-8" />',
        '<meta name="viewport" content="width=device-width, initial-scale=1" />',
        "<title>KF Reference Comparison Report</title>",
        "<style>",
        ":root { color-scheme: light; }",
        "body { font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, sans-serif; margin: 24px; background: #fafafa; color: #111; }",
        "h1 { margin: 0 0 6px 0; }",
        ".sub { color: #444; margin: 0 0 18px 0; }",
        ".bag { background: #fff; border: 1px solid #ddd; border-radius: 14px; padding: 16px; margin-bottom: 18px; }",
        ".meta { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 10px; margin: 12px 0; }",
        ".cell { background: #f5f5f5; border-radius: 10px; padding: 10px 12px; }",
        ".k { display: block; color: #666; font-size: 12px; margin-bottom: 4px; }",
        ".v { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 14px; }",
        ".imgs { display: grid; grid-template-columns: 1fr; gap: 14px; margin-top: 14px; }",
        "img { width: 100%; border: 1px solid #e5e5e5; border-radius: 12px; background: #fff; }",
        ".missing { color: #888; font-style: italic; padding: 12px; border: 1px dashed #ccc; border-radius: 10px; }",
        "a { color: #0645ad; }",
        "code { background: #f3f3f3; padding: 2px 6px; border-radius: 6px; }",
        "</style>",
        "</head>",
        "<body>",
        "<h1>KF Reference Comparison Report</h1>",
        '<p class="sub">estimated と reference を比較する XY 軌跡 / z+RPY 時系列。reference は各 run の <code>ground_truth.csv</code> を使用。</p>',
    ]

    for row in rows:
        bag_dir = Path(row["output_dir"])
        run_row = read_first_row(bag_dir / "summary.csv")
        run_id = run_row.get("run_id", "run_001")
        per_bag_reports = sorted(bag_dir.glob("open_data_report_*.html"))
        per_bag_report = per_bag_reports[-1] if per_bag_reports else None
        xy = bag_dir / run_id / f"{run_id}_trajectory_xy.png"
        ts = bag_dir / run_id / f"{run_id}_timeseries_z_rpy.png"

        def img_or_missing(path: Path, alt: str) -> str:
            if path.exists():
                return f'<img src="{esc(rel(path))}" alt="{esc(alt)}" loading="lazy" />'
            return '<div class="missing">(missing)</div>'

        body.extend(
            [
                '<section class="bag">',
                f"<h2>{esc(row.get('bag', ''))}</h2>",
                '<div class="sub">'
                f"profile: <code>{esc(row.get('profile', ''))}</code>"
                + (
                    f' | per-bag report: <a href="{esc(rel(per_bag_report))}">{esc(per_bag_report.name)}</a>'
                    if per_bag_report is not None
                    else ""
                )
                + "</div>",
                '<div class="meta">',
                f'<div class="cell"><span class="k">rmse_3d_m</span><span class="v">{esc(row.get("rmse_3d_m", ""))}</span></div>',
                f'<div class="cell"><span class="k">rmse_3d_nobias_m</span><span class="v">{esc(row.get("rmse_3d_nobias_m", ""))}</span></div>',
                f'<div class="cell"><span class="k">yaw_rmse_deg</span><span class="v">{esc(row.get("yaw_rmse_deg", ""))}</span></div>',
                f'<div class="cell"><span class="k">matched_samples</span><span class="v">{esc(row.get("matched_samples", ""))}</span></div>',
                f'<div class="cell"><span class="k">threshold_status</span><span class="v">{esc(row.get("threshold_status", ""))}</span></div>',
                f'<div class="cell"><span class="k">initial_yaw_label</span><span class="v">{esc(row.get("initial_yaw_label", ""))}</span></div>',
                "</div>",
                '<div class="imgs">',
                img_or_missing(xy, f"{row.get('bag', '')} XY trajectory"),
                img_or_missing(ts, f"{row.get('bag', '')} z and RPY time series"),
                "</div>",
                "</section>",
            ]
        )

    body.extend(["</body>", "</html>"])
    report_path.write_text("\n".join(body) + "\n", encoding="utf-8")
    return report_path


def main() -> int:
    args = build_parser().parse_args()
    ws_root = args.ws_root.resolve()
    data_dir = (ws_root / args.data_dir).resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    thresholds = load_thresholds(args.thresholds_json.resolve() if args.thresholds_json else None)

    rows: List[Dict[str, str]] = []
    failure_count = 0

    for bag_name in args.bags:
        bag_path = data_dir / bag_name
        if not bag_path.exists():
            print(f"skip: bag not found: {bag_path}", file=sys.stderr)
            continue

        profile_name, params = choose_profile(bag_name)
        bag_out_dir = output_dir / bag_name
        bag_out_dir.mkdir(parents=True, exist_ok=True)
        grid_path = bag_out_dir / "selected_param_grid.json"
        write_single_grid(grid_path, params)

        cmd = [
            sys.executable,
            str(SWEEP_SCRIPT),
            "--bag-path",
            str(bag_path),
            "--param-grid-json",
            str(grid_path),
            "--output-dir",
            str(bag_out_dir),
            "--play-rate",
            f"{args.play_rate:.12g}",
            "--imu-topic",
            "/ins_imu",
            "--gnss-topic",
            "/gnss_pose",
            "--ground-truth-topic",
            "/ins_pose",
            "--play-topics",
            "/sensing/imu/imu_data",
            "/gnss/fix",
            "/lvx_client/gsof/ins_solution_49",
            "--enable-navsatfix-to-pose",
            "--navsatfix-input-topic",
            "/gnss/fix",
            "--navsatfix-output-topic",
            "/gnss_pose",
            "--enable-applanix-to-pose",
            "--applanix-input-topic",
            "/lvx_client/gsof/ins_solution_49",
            "--applanix-output-topic",
            "/ins_pose",
            "--applanix-origin-navsatfix-topic",
            "/gnss/fix",
            "--enable-applanix-to-imu",
            "--applanix-imu-output-topic",
            "/ins_imu",
            "--applanix-imu-output-mode",
            "ros",
            "--initial-yaw-source-topic",
            "/ins_pose",
            "--initial-yaw-source-msg-type",
            "pose_stamped",
            "--plot-best",
        ]

        print(f"\n=== {bag_name} :: {profile_name} ===")
        proc = subprocess.run(cmd, cwd=ws_root, check=False)

        summary_csv = bag_out_dir / "summary.csv"
        row: Dict[str, str] = {
            "bag": bag_name,
            "profile": profile_name,
            "output_dir": str(bag_out_dir),
            "grid_path": str(grid_path),
            "exit_code": str(proc.returncode),
            "rmse_3d_threshold_m": "",
            "rmse_3d_margin_m": "",
            "threshold_status": "n/a",
        }
        if summary_csv.exists():
            first = read_first_row(summary_csv)
            if first:
                for key in (
                    "status",
                    "rmse_3d_m",
                    "rmse_3d_nobias_m",
                    "yaw_rmse_deg",
                    "initial_yaw_label",
                    "matched_samples",
                ):
                    row[key] = first.get(key, "")
        if proc.returncode != 0 or row.get("status") not in {"ok", ""}:
            failure_count += 1
        threshold_cfg = thresholds.get(bag_name)
        if threshold_cfg is not None:
            threshold = float(threshold_cfg["max_rmse_3d_m"])
            row["rmse_3d_threshold_m"] = f"{threshold:.12g}"
            try:
                rmse_3d = float(row.get("rmse_3d_m", "nan"))
            except ValueError:
                rmse_3d = float("nan")
            if math.isfinite(rmse_3d):
                margin = threshold - rmse_3d
                row["rmse_3d_margin_m"] = f"{margin:.12g}"
                if rmse_3d <= threshold:
                    row["threshold_status"] = "pass"
                else:
                    row["threshold_status"] = "fail"
                    failure_count += 1
            else:
                row["threshold_status"] = "missing"
                failure_count += 1
        rows.append(row)

    summary_path = output_dir / "profile_validation_summary.csv"
    fieldnames = [
        "bag",
        "profile",
        "output_dir",
        "grid_path",
        "exit_code",
        "status",
        "rmse_3d_m",
        "rmse_3d_nobias_m",
        "yaw_rmse_deg",
        "initial_yaw_label",
        "matched_samples",
        "rmse_3d_threshold_m",
        "rmse_3d_margin_m",
        "threshold_status",
    ]
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    report_path = write_html_report(output_dir, rows)

    print(f"\nsummary_csv: {summary_path}")
    print(f"report_html: {report_path}")
    if failure_count > 0:
        print(f"validation_failures: {failure_count}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
