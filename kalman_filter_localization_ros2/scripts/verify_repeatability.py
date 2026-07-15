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
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Verify exact-input and sample-level repeatability across experiment manifests."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


FIELDS = ('x', 'y', 'z', 'yaw')


def angle_difference(left, right):
    """Return the magnitude of the shortest angular difference."""
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def find_run(manifest, profile):
    """Return one named run or fail on missing/duplicate profile entries."""
    runs = [run for run in manifest.get('runs', []) if run.get('name') == profile]
    if len(runs) != 1:
        raise ValueError(
            "manifest must contain exactly one '{}' run, found {}".format(
                profile, len(runs)))
    return runs[0]


def read_manifest(path, profile):
    """Load a completed manifest and resolve its estimate CSV."""
    path = Path(path)
    manifest = json.loads(path.read_text(encoding='utf-8'))
    run = find_run(manifest, profile)
    result = run.get('result', {})
    if not manifest.get('completed') or not result.get('completed'):
        raise ValueError('incomplete experiment manifest: {}'.format(path))
    count_check = result.get('input_count_check', {})
    if not count_check.get('passed'):
        raise ValueError('input count gate did not pass: {}'.format(path))
    estimate = result.get('estimate_csv', {})
    estimate_path = Path(estimate.get('path', ''))
    if not estimate_path.is_file():
        raise ValueError('missing estimate CSV for manifest: {}'.format(path))
    return {
        'path': str(path.resolve()),
        'manifest': manifest,
        'run': run,
        'result': result,
        'estimate_path': estimate_path,
    }


def comparable_provenance(reference, candidate):
    """Find immutable inputs or merged configuration that differ between runs."""
    differences = {}
    reference_inputs = reference['manifest'].get('inputs', {})
    candidate_inputs = candidate['manifest'].get('inputs', {})
    for name in ('bag', 'reference', 'base_profile'):
        left = reference_inputs.get(name, {}).get('sha256')
        right = candidate_inputs.get(name, {}).get('sha256')
        if left != right:
            differences['input.' + name] = {'reference': left, 'candidate': right}
    left = reference['run'].get('config_artifact', {}).get('sha256')
    right = candidate['run'].get('config_artifact', {}).get('sha256')
    if left != right:
        differences['merged_config'] = {'reference': left, 'candidate': right}
    reference_runtime = reference['manifest'].get('runtime_artifacts', {})
    candidate_runtime = candidate['manifest'].get('runtime_artifacts', {})
    for name in ('node', 'component', 'runner'):
        left = reference_runtime.get(name, {}).get('sha256')
        right = candidate_runtime.get(name, {}).get('sha256')
        if left is None or right is None or left != right:
            differences['runtime.' + name] = {
                'reference': left, 'candidate': right}
    for name in ('rate', 'startup_delay', 'max_reference_gap', 'time_offset'):
        left = reference['manifest'].get(name)
        right = candidate['manifest'].get(name)
        if left != right:
            differences[name] = {'reference': left, 'candidate': right}
    return differences


def compare_csv(reference_path, candidate_path, stamp_tolerance, value_tolerance):
    """Compare two trajectories in lockstep without interpolation or alignment."""
    maximum = {'stamp': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    first_mismatch = None
    reference_count = 0
    candidate_count = 0
    with open(reference_path, newline='', encoding='utf-8') as reference_stream, \
            open(candidate_path, newline='', encoding='utf-8') as candidate_stream:
        reference_rows = csv.DictReader(reference_stream)
        candidate_rows = csv.DictReader(candidate_stream)
        while True:
            reference_row = next(reference_rows, None)
            candidate_row = next(candidate_rows, None)
            if reference_row is None and candidate_row is None:
                break
            if reference_row is not None:
                reference_count += 1
            if candidate_row is not None:
                candidate_count += 1
            if reference_row is None or candidate_row is None:
                if first_mismatch is None:
                    first_mismatch = {
                        'sample_index': max(reference_count, candidate_count),
                        'reason': 'sample_count',
                    }
                continue
            differences = {
                'stamp': abs(
                    float(reference_row['stamp']) - float(candidate_row['stamp'])),
                'x': abs(float(reference_row['x']) - float(candidate_row['x'])),
                'y': abs(float(reference_row['y']) - float(candidate_row['y'])),
                'z': abs(float(reference_row['z']) - float(candidate_row['z'])),
                'yaw': angle_difference(
                    float(reference_row['yaw']), float(candidate_row['yaw'])),
            }
            for name, difference in differences.items():
                maximum[name] = max(maximum[name], difference)
            mismatched = differences['stamp'] > stamp_tolerance or any(
                differences[name] > value_tolerance for name in FIELDS)
            if mismatched and first_mismatch is None:
                first_mismatch = {
                    'sample_index': reference_count,
                    'reference_stamp': float(reference_row['stamp']),
                    'candidate_stamp': float(candidate_row['stamp']),
                    'absolute_difference': differences,
                }
    return {
        'passed': first_mismatch is None,
        'reference_samples': reference_count,
        'candidate_samples': candidate_count,
        'maximum_absolute_difference': maximum,
        'first_mismatch': first_mismatch,
    }


def verify(manifest_paths, profile, stamp_tolerance=0.0, value_tolerance=1.0e-12):
    """Verify provenance and trajectories for at least two repeated runs."""
    if len(manifest_paths) < 2:
        raise ValueError('at least two manifests are required')
    records = [read_manifest(path, profile) for path in manifest_paths]
    reference = records[0]
    comparisons = []
    for candidate in records[1:]:
        provenance_differences = comparable_provenance(reference, candidate)
        trajectory = compare_csv(
            reference['estimate_path'], candidate['estimate_path'],
            stamp_tolerance, value_tolerance)
        comparisons.append({
            'reference_manifest': reference['path'],
            'candidate_manifest': candidate['path'],
            'provenance_differences': provenance_differences,
            'trajectory': trajectory,
            'passed': not provenance_differences and trajectory['passed'],
        })
    return {
        'profile': profile,
        'stamp_tolerance': stamp_tolerance,
        'value_tolerance': value_tolerance,
        'comparisons': comparisons,
        'passed': all(comparison['passed'] for comparison in comparisons),
    }


def main(argv=None):
    """Run the repeatability gate and optionally write its JSON evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifests', nargs='+', type=Path)
    parser.add_argument('--profile', default='baseline')
    parser.add_argument('--stamp-tolerance', type=float, default=0.0)
    parser.add_argument('--value-tolerance', type=float, default=1.0e-12)
    parser.add_argument('--output-json', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.stamp_tolerance < 0.0 or args.value_tolerance < 0.0:
            raise ValueError('tolerances must be non-negative')
        report = verify(
            args.manifests, args.profile, args.stamp_tolerance,
            args.value_tolerance)
        text = json.dumps(report, indent=2) + '\n'
        if args.output_json:
            args.output_json.parent.mkdir(parents=True, exist_ok=True)
            args.output_json.write_text(text, encoding='utf-8')
        print(text, end='')
        return 0 if report['passed'] else 1
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
