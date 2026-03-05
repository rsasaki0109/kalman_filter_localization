#!/usr/bin/env python3
"""Inject one or more sweep summary.csv files into agent_design_loop_report.html."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import html
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

START_MARKER = "<!-- AUTO_RESULTS_START -->"
END_MARKER = "<!-- AUTO_RESULTS_END -->"


@dataclass
class CycleMetrics:
    label: str
    source: Path
    total: int
    ok: int
    failed: int
    best: Optional[Dict[str, str]]
    best_nobias: Optional[Dict[str, str]]
    top: List[Dict[str, str]]
    failed_status_counts: Dict[str, int]
    ok_rows: List[Dict[str, str]]


def to_float(value: object) -> Optional[float]:
    try:
        if value is None:
            return None
        text = str(value).strip()
        if not text:
            return None
        return float(text)
    except Exception:
        return None


def fmt(value: Optional[float], digits: int = 6) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def load_rows(summary_csv: Path) -> List[Dict[str, str]]:
    with summary_csv.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def compute_cycle_metrics(
    rows: List[Dict[str, str]],
    *,
    source: Path,
    label: str,
    top_n: int,
) -> CycleMetrics:
    total = len(rows)
    ok_rows = [r for r in rows if str(r.get("status", "")).strip() == "ok"]
    failed_rows = [r for r in rows if str(r.get("status", "")).strip() != "ok"]
    failed_status_counts = Counter(str(r.get("status", "")).strip() or "unknown" for r in failed_rows)

    ok_with_rmse = [r for r in ok_rows if to_float(r.get("rmse_3d_m")) is not None]
    ok_with_nobias = [r for r in ok_rows if to_float(r.get("rmse_3d_nobias_m")) is not None]
    top = sorted(ok_with_rmse, key=lambda r: float(r["rmse_3d_m"]))[: max(1, top_n)]
    best = top[0] if top else None
    best_nobias = (
        min(ok_with_nobias, key=lambda r: float(r["rmse_3d_nobias_m"])) if ok_with_nobias else None
    )

    return CycleMetrics(
        label=label,
        source=source,
        total=total,
        ok=len(ok_rows),
        failed=total - len(ok_rows),
        best=best,
        best_nobias=best_nobias,
        top=top,
        failed_status_counts=dict(failed_status_counts),
        ok_rows=ok_rows,
    )


def build_recommendations(cycles: List[CycleMetrics]) -> List[str]:
    recommendations: List[str] = []
    if not cycles:
        return recommendations

    total_runs = sum(c.total for c in cycles)
    total_failed = sum(c.failed for c in cycles)
    failure_rate = (total_failed / total_runs) if total_runs > 0 else 0.0
    if failure_rate > 0.5:
        recommendations.append(
            "失敗率が高いので、timestamp整合 (`time_normalize` / `time_align`) と `max_imu_dt_sec` を優先確認する。"
        )

    best_candidates = [c.best for c in cycles if c.best is not None]
    if best_candidates:
        global_best = min(best_candidates, key=lambda r: float(r["rmse_3d_m"]))
        gap = to_float(global_best.get("rmse_3d_m")) or 0.0
        yaw = to_float(global_best.get("yaw_rmse_deg")) or 0.0
        if gap > 0.5:
            recommendations.append(
                "best rmse_3d_m がまだ高いため、`var_gnss_xy` と `var_imu_acc` の探索範囲を広げる。"
            )
        if yaw > 0.05:
            recommendations.append(
                "yaw誤差が残るため、`var_imu_orientation_rpy` と `var_imu_w` を再探索し、姿勢重みを最適化する。"
            )

    if not recommendations:
        recommendations.append("現状は安定しているため、同条件で別bagへ横展開して再現性を確認する。")
    return recommendations


def build_auto_section(cycles: List[CycleMetrics], *, top_n_overall: int) -> str:
    generated_at = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    all_ok_rows: List[tuple[str, Dict[str, str]]] = []
    for cycle in cycles:
        for row in cycle.ok_rows:
            if to_float(row.get("rmse_3d_m")) is not None:
                all_ok_rows.append((cycle.label, row))

    global_top = sorted(all_ok_rows, key=lambda item: float(item[1]["rmse_3d_m"]))[
        : max(1, top_n_overall)
    ]

    lines: List[str] = []
    lines.append('      <section class="card">')
    lines.append("        <h2>Latest Metrics (Auto)</h2>")
    lines.append(
        "        <p class=\"muted\">"
        f"generated_at={generated_at} / cycles={len(cycles)} / sources={len(cycles)} files"
        "</p>"
    )
    lines.append("      </section>")

    lines.append('      <section class="card">')
    lines.append("        <h2>Cycle Summary</h2>")
    lines.append("        <table>")
    lines.append(
        "          <thead><tr><th>cycle</th><th>source</th><th>ok</th><th>failed</th><th>best_run</th><th>best_rmse_3d_m</th><th>best_yaw_rmse_deg</th></tr></thead>"
    )
    lines.append("          <tbody>")
    for cycle in cycles:
        best_run = html.escape(str(cycle.best.get("run_id", "-"))) if cycle.best else "-"
        best_rmse = fmt(to_float(cycle.best.get("rmse_3d_m")) if cycle.best else None, 6)
        best_yaw = fmt(to_float(cycle.best.get("yaw_rmse_deg")) if cycle.best else None, 6)
        lines.append(
            "            <tr>"
            f"<td>{html.escape(cycle.label)}</td>"
            f"<td><code>{html.escape(str(cycle.source))}</code></td>"
            f"<td>{cycle.ok}</td>"
            f"<td>{cycle.failed}</td>"
            f"<td>{best_run}</td>"
            f"<td>{best_rmse}</td>"
            f"<td>{best_yaw}</td>"
            "</tr>"
        )
    lines.append("          </tbody>")
    lines.append("        </table>")
    lines.append("      </section>")

    lines.append('      <section class="card">')
    lines.append(f"        <h2>Overall Top {max(1, top_n_overall)} by rmse_3d_m</h2>")
    lines.append("        <table>")
    lines.append(
        "          <thead><tr><th>rank</th><th>cycle</th><th>run_id</th><th>rmse_3d_m</th><th>rmse_3d_nobias_m</th><th>yaw_rmse_deg</th><th>var_imu_orientation_rpy</th><th>var_imu_w</th><th>var_imu_acc</th><th>var_gnss_xy</th><th>var_gnss_z</th><th>max_imu_dt_sec</th></tr></thead>"
    )
    lines.append("          <tbody>")
    for idx, (label, row) in enumerate(global_top, start=1):
        lines.append(
            "            <tr>"
            f"<td>{idx}</td>"
            f"<td>{html.escape(label)}</td>"
            f"<td>{html.escape(str(row.get('run_id', '-')))}</td>"
            f"<td>{fmt(to_float(row.get('rmse_3d_m')), 6)}</td>"
            f"<td>{fmt(to_float(row.get('rmse_3d_nobias_m')), 6)}</td>"
            f"<td>{fmt(to_float(row.get('yaw_rmse_deg')), 6)}</td>"
            f"<td>{html.escape(str(row.get('var_imu_orientation_rpy', '-')))}</td>"
            f"<td>{html.escape(str(row.get('var_imu_w', '-')))}</td>"
            f"<td>{html.escape(str(row.get('var_imu_acc', '-')))}</td>"
            f"<td>{html.escape(str(row.get('var_gnss_xy', '-')))}</td>"
            f"<td>{html.escape(str(row.get('var_gnss_z', '-')))}</td>"
            f"<td>{html.escape(str(row.get('max_imu_dt_sec', '-')))}</td>"
            "</tr>"
        )
    lines.append("          </tbody>")
    lines.append("        </table>")
    lines.append("      </section>")

    lines.append('      <section class="card">')
    lines.append("        <h2>Auto Recommendations</h2>")
    lines.append("        <ul>")
    for rec in build_recommendations(cycles):
        lines.append(f"          <li>{html.escape(rec)}</li>")
    lines.append("        </ul>")
    lines.append("      </section>")

    for cycle in cycles:
        lines.append('      <section class="card">')
        lines.append(f"        <h2>Cycle Detail: {html.escape(cycle.label)}</h2>")
        lines.append(
            f"        <p class=\"muted\">source=<code>{html.escape(str(cycle.source))}</code></p>"
        )
        if cycle.failed_status_counts:
            lines.append("        <p>")
            for status, count in sorted(cycle.failed_status_counts.items(), key=lambda item: item[0]):
                lines.append(
                    f"          <span class=\"pill warn\">{html.escape(status)} {count}</span>"
                )
            lines.append("        </p>")
        if cycle.top:
            lines.append("        <table>")
            lines.append(
                "          <thead><tr><th>rank</th><th>run_id</th><th>rmse_3d_m</th><th>rmse_3d_nobias_m</th><th>yaw_rmse_deg</th><th>matched_samples</th></tr></thead>"
            )
            lines.append("          <tbody>")
            for idx, row in enumerate(cycle.top, start=1):
                lines.append(
                    "            <tr>"
                    f"<td>{idx}</td>"
                    f"<td>{html.escape(str(row.get('run_id', '-')))}</td>"
                    f"<td>{fmt(to_float(row.get('rmse_3d_m')), 6)}</td>"
                    f"<td>{fmt(to_float(row.get('rmse_3d_nobias_m')), 6)}</td>"
                    f"<td>{fmt(to_float(row.get('yaw_rmse_deg')), 6)}</td>"
                    f"<td>{fmt(to_float(row.get('matched_samples')), 0)}</td>"
                    "</tr>"
                )
            lines.append("          </tbody>")
            lines.append("        </table>")
        else:
            lines.append("        <p>No successful runs in this cycle.</p>")
        lines.append("      </section>")
    return "\n".join(lines)


def replace_between_markers(text: str, replacement: str) -> str:
    start_idx = text.find(START_MARKER)
    end_idx = text.find(END_MARKER)
    if start_idx < 0 or end_idx < 0 or end_idx <= start_idx:
        raise RuntimeError("auto markers not found in report html")
    before = text[: start_idx + len(START_MARKER)]
    after = text[end_idx:]
    return before + "\n" + replacement + "\n    " + after


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--summary-csv", action="append", required=True, type=Path)
    p.add_argument("--cycle-label", action="append")
    p.add_argument(
        "--report-html",
        type=Path,
        default=Path("src/kalman_filter_localization/docs/agent_design_loop_report.html"),
    )
    p.add_argument("--top-n", type=int, default=5)
    p.add_argument("--top-n-overall", type=int, default=10)
    return p.parse_args()


def resolve_labels(paths: List[Path], labels: Optional[List[str]]) -> List[str]:
    if not labels:
        return [p.parent.name for p in paths]
    if len(labels) == 1 and len(paths) > 1:
        prefix = labels[0]
        return [f"{prefix}_{idx:02d}" for idx, _ in enumerate(paths, start=1)]
    if len(labels) != len(paths):
        raise RuntimeError("--cycle-label count must be 1 or match --summary-csv count")
    return labels


def main() -> int:
    args = parse_args()
    labels = resolve_labels(args.summary_csv, args.cycle_label)
    metrics: List[CycleMetrics] = []
    for summary_csv, label in zip(args.summary_csv, labels):
        rows = load_rows(summary_csv)
        metrics.append(
            compute_cycle_metrics(rows, source=summary_csv, label=label, top_n=max(1, args.top_n))
        )

    report_text = args.report_html.read_text(encoding="utf-8")
    auto_html = build_auto_section(metrics, top_n_overall=max(1, args.top_n_overall))
    updated = replace_between_markers(report_text, auto_html)
    args.report_html.write_text(updated, encoding="utf-8")
    print(f"updated_report: {args.report_html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
