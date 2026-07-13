#!/usr/bin/env python3
# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Compare localization result CSV files against one reference trajectory."""

import argparse
import csv
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec
from importlib.util import spec_from_loader
import math
from pathlib import Path


def load_evaluator():
    """Load the evaluator from a source tree or an installed ROS executable."""
    script_directory = Path(__file__).resolve().parent
    source_path = script_directory / 'evaluate_localization.py'
    installed_path = script_directory / 'evaluate_localization'
    path = source_path if source_path.exists() else installed_path
    loader = SourceFileLoader('evaluate_localization', str(path))
    spec = spec_from_loader(loader.name, loader)
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


EVALUATOR = load_evaluator()
evaluate = EVALUATOR.evaluate
read_csv = EVALUATOR.read_csv


METRICS = [
    'rmse_3d_m',
    'rmse_horizontal_m',
    'rmse_vertical_m',
    'yaw_rmse_deg',
    'max_error_3d_m',
    'match_ratio',
]


def parse_estimate(value):
    """Parse NAME=PATH command-line values."""
    if '=' not in value:
        raise argparse.ArgumentTypeError('estimate must use NAME=PATH')
    name, path = value.split('=', 1)
    if not name or not path:
        raise argparse.ArgumentTypeError('estimate must use non-empty NAME=PATH')
    return name, path


def compare(
        reference_path, estimates, max_reference_gap, time_offset,
        align_translation=False, start_stamp=None, end_stamp=None):
    """Evaluate named trajectories and calculate deltas from the first result."""
    references = read_csv(reference_path)
    rows = []
    for name, path in estimates:
        summary, unused_errors = evaluate(
            read_csv(path), references, max_reference_gap, time_offset,
            align_translation, start_stamp, end_stamp)
        row = {'name': name}
        row.update({metric: summary[metric] for metric in METRICS})
        rows.append(row)
    if not rows:
        raise ValueError('at least one estimate is required')
    baseline = rows[0]
    for row in rows:
        for metric in METRICS:
            baseline_value = baseline[metric]
            value = row[metric]
            row['delta_' + metric] = (
                value - baseline_value
                if value is not None and baseline_value is not None else None)
    return rows


def write_csv(path, rows):
    """Write machine-readable comparison results."""
    fields = ['name'] + METRICS + ['delta_' + metric for metric in METRICS]
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def format_value(value, digits=4):
    """Format optional finite metric values for Markdown."""
    return 'n/a' if value is None or not math.isfinite(value) else f'{value:.{digits}f}'


def write_markdown(path, rows):
    """Write a compact human-readable ablation table."""
    lines = [
        '| Profile | 3D RMSE m | Delta m | Horizontal m | Yaw RMSE deg | Max 3D m | Match |',
        '|---|---:|---:|---:|---:|---:|---:|',
    ]
    for row in rows:
        lines.append(
            '| {} | {} | {} | {} | {} | {} | {} |'.format(
                row['name'],
                format_value(row['rmse_3d_m']),
                format_value(row['delta_rmse_3d_m']),
                format_value(row['rmse_horizontal_m']),
                format_value(row['yaw_rmse_deg']),
                format_value(row['max_error_3d_m']),
                format_value(row['match_ratio'], 3)))
    Path(path).write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    """Run the comparison command."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-csv', required=True)
    parser.add_argument('--estimate', action='append', type=parse_estimate, required=True)
    parser.add_argument('--max-reference-gap', type=float, default=0.2)
    parser.add_argument('--time-offset', type=float, default=0.0)
    parser.add_argument('--align-translation', action='store_true')
    parser.add_argument('--start-stamp', type=float)
    parser.add_argument('--end-stamp', type=float)
    parser.add_argument('--output-csv', required=True)
    parser.add_argument('--output-markdown', required=True)
    args = parser.parse_args(argv)
    try:
        rows = compare(
            args.reference_csv, args.estimate, args.max_reference_gap,
            args.time_offset, args.align_translation, args.start_stamp, args.end_stamp)
        write_csv(args.output_csv, rows)
        write_markdown(args.output_markdown, rows)
        return 0
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
