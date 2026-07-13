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

import contextlib
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import yaml


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'evaluate_localization.py'
COMPARISON_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'compare_localization_results.py')
RUNNER_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'run_urbannav_ablation.py')
PREPARE_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'prepare_urbannav_tokyo.py')
DATA = Path(__file__).parent / 'data'
SPEC = importlib.util.spec_from_file_location('evaluate_localization', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
COMPARISON_SPEC = importlib.util.spec_from_file_location(
    'compare_localization_results', COMPARISON_SCRIPT)
COMPARISON_MODULE = importlib.util.module_from_spec(COMPARISON_SPEC)
with mock.patch.dict('sys.modules', {'evaluate_localization': MODULE}):
    COMPARISON_SPEC.loader.exec_module(COMPARISON_MODULE)
RUNNER_SPEC = importlib.util.spec_from_file_location(
    'run_urbannav_ablation', RUNNER_SCRIPT)
RUNNER_MODULE = importlib.util.module_from_spec(RUNNER_SPEC)
RUNNER_SPEC.loader.exec_module(RUNNER_MODULE)
PREPARE_SPEC = importlib.util.spec_from_file_location(
    'prepare_urbannav_tokyo', PREPARE_SCRIPT)
PREPARE_MODULE = importlib.util.module_from_spec(PREPARE_SPEC)
PREPARE_SPEC.loader.exec_module(PREPARE_MODULE)


class EvaluateLocalizationTest(unittest.TestCase):
    def test_fixed_regression_dataset_metrics(self):
        estimates = MODULE.read_csv(DATA / 'estimate.csv')
        references = MODULE.read_csv(DATA / 'reference.csv')
        summary, unused_errors = MODULE.evaluate(estimates, references, 0.2)
        self.assertEqual(summary['matched_samples'], 5)
        self.assertAlmostEqual(summary['match_ratio'], 1.0)
        self.assertAlmostEqual(summary['rmse_3d_m'], 0.0749666592559653)
        self.assertAlmostEqual(summary['rmse_horizontal_m'], 0.07429670248402688)
        self.assertAlmostEqual(summary['rmse_vertical_m'], 0.01)
        self.assertAlmostEqual(summary['yaw_rmse_deg'], math.sqrt(2.0))
        self.assertAlmostEqual(summary['max_error_3d_m'], 0.1048808848170152)

    def test_fixed_regression_dataset_passes_acceptance_limits(self):
        with contextlib.redirect_stdout(io.StringIO()):
            result = MODULE.main([
                '--estimate-csv', str(DATA / 'estimate.csv'),
                '--reference-csv', str(DATA / 'reference.csv'),
                '--max-rmse-3d', '0.08',
                '--max-yaw-rmse-deg', '1.5',
                '--max-error-3d', '0.11',
                '--min-match-ratio', '1.0',
            ])
        self.assertEqual(result, 0)

    def test_interpolation_and_metrics(self):
        references = [MODULE.Sample(0.0, 0.0, 0.0, 0.0, 0.0),
                      MODULE.Sample(1.0, 1.0, 0.0, 0.0, math.pi / 2)]
        estimates = [MODULE.Sample(0.5, 0.8, 0.4, 0.1, math.pi / 4 + math.radians(2))]
        summary, errors = MODULE.evaluate(estimates, references, 2.0)
        self.assertEqual(summary['matched_samples'], 1)
        self.assertAlmostEqual(summary['rmse_3d_m'], math.sqrt(0.26))
        self.assertAlmostEqual(summary['yaw_rmse_deg'], 2.0)
        self.assertAlmostEqual(errors[0]['horizontal'], 0.5)

    def test_yaw_interpolation_wraps_at_pi(self):
        references = [MODULE.Sample(0.0, 0.0, 0.0, 0.0, math.radians(179)),
                      MODULE.Sample(1.0, 0.0, 0.0, 0.0, math.radians(-179))]
        result = MODULE.interpolate(references, [0.0, 1.0], 0.5, 2.0)
        self.assertAlmostEqual(abs(MODULE.normalize_angle(result.yaw)), math.pi)

    def test_large_reference_gap_is_rejected(self):
        references = [MODULE.Sample(0.0, 0.0, 0.0, 0.0, 0.0),
                      MODULE.Sample(2.0, 0.0, 0.0, 0.0, 0.0)]
        estimates = [MODULE.Sample(1.0, 0.0, 0.0, 0.0, 0.0)]
        with self.assertRaisesRegex(ValueError, 'no overlapping samples'):
            MODULE.evaluate(estimates, references, 0.2)

    def test_thresholds_report_upper_and_lower_limit_failures(self):
        summary = {'rmse_3d_m': 0.25, 'match_ratio': 0.8}
        failures = MODULE.check_thresholds(
            summary, {'rmse_3d_m': 0.2, 'match_ratio': 0.9})
        self.assertEqual(len(failures), 2)
        self.assertIn('exceeds', failures[0])
        self.assertIn('below', failures[1])

    def test_unconfigured_thresholds_are_ignored(self):
        failures = MODULE.check_thresholds({'rmse_3d_m': 10.0}, {'rmse_3d_m': None})
        self.assertEqual(failures, [])

    def test_cli_returns_one_and_writes_failed_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / 'reference.csv'
            estimate = root / 'estimate.csv'
            output = root / 'metrics.json'
            reference.write_text(
                'stamp,x,y,z,yaw\n0,0,0,0,0\n1,1,0,0,0\n', encoding='utf-8')
            estimate.write_text(
                'stamp,x,y,z,yaw\n0.5,0.7,0,0,0\n', encoding='utf-8')
            with contextlib.redirect_stdout(io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    result = MODULE.main([
                        '--estimate-csv', str(estimate),
                        '--reference-csv', str(reference),
                        '--max-reference-gap', '2.0',
                        '--max-rmse-3d', '0.1',
                        '--output-json', str(output),
                    ])
            self.assertEqual(result, 1)
            self.assertIn('"passed": false', output.read_text(encoding='utf-8'))

    def test_ablation_comparison_uses_first_estimate_as_baseline(self):
        estimates = [
            ('baseline', DATA / 'estimate.csv'),
            ('candidate', DATA / 'estimate.csv'),
        ]
        rows = COMPARISON_MODULE.compare(
            DATA / 'reference.csv', estimates, 0.2, 0.0)
        self.assertEqual([row['name'] for row in rows], ['baseline', 'candidate'])
        self.assertAlmostEqual(rows[0]['delta_rmse_3d_m'], 0.0)
        self.assertAlmostEqual(rows[1]['delta_rmse_3d_m'], 0.0)

    def test_ablation_comparison_writes_csv_and_markdown(self):
        rows = COMPARISON_MODULE.compare(
            DATA / 'reference.csv', [('baseline', DATA / 'estimate.csv')],
            0.2, 0.0)
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / 'comparison.csv'
            markdown_path = Path(directory) / 'comparison.md'
            COMPARISON_MODULE.write_csv(csv_path, rows)
            COMPARISON_MODULE.write_markdown(markdown_path, rows)
            self.assertIn('delta_rmse_3d_m', csv_path.read_text(encoding='utf-8'))
            markdown = markdown_path.read_text(encoding='utf-8')
            self.assertIn('| Profile |', markdown)
            self.assertIn('| baseline |', markdown)

    def test_urbannav_runner_deep_merges_research_parameters(self):
        base = {'ekf_localization': {'ros__parameters': {
            'use_gnss': True, 'use_zupt': False}}}
        override = {'ekf_localization': {'ros__parameters': {
            'use_zupt': True}}}
        merged = RUNNER_MODULE.deep_merge(base, override)
        parameters = merged['ekf_localization']['ros__parameters']
        self.assertTrue(parameters['use_gnss'])
        self.assertTrue(parameters['use_zupt'])
        self.assertFalse(base['ekf_localization']['ros__parameters']['use_zupt'])

    def test_urbannav_runner_dry_run_writes_configs_and_manifest(self):
        profiles = Path(__file__).parents[1] / 'param' / 'profiles'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_bag = root / 'input_bag'
            input_bag.mkdir()
            output = root / 'results'
            result = RUNNER_MODULE.main([
                '--input-bag', str(input_bag),
                '--reference-csv', str(DATA / 'reference.csv'),
                '--output-dir', str(output),
                '--base-profile', str(profiles / 'urbannav_tokyo_tuned.yaml'),
                '--profiles-dir', str(profiles),
                '--dry-run',
            ])
            self.assertEqual(result, 0)
            manifest = json.loads(
                (output / 'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(
                [run['name'] for run in manifest['runs']],
                ['baseline', 'nhc', 'robust', 'full'])
            full = yaml.safe_load(
                (output / 'configs' / 'full.yaml').read_text(encoding='utf-8'))
            parameters = full['ekf_localization']['ros__parameters']
            self.assertTrue(parameters['use_gnss'])
            self.assertTrue(parameters['use_zupt'])

    def test_urbannav_geodetic_origin_maps_to_zero_enu(self):
        origin = (35.62931853, 139.78712595, 44.6995)
        ecef = PREPARE_MODULE.geodetic_to_ecef(*origin)
        enu = PREPARE_MODULE.ecef_to_enu(
            ecef, ecef, origin[0], origin[1])
        self.assertEqual(enu, (0.0, 0.0, 0.0))

    def test_urbannav_gps_stamp_is_continuous_across_weeks(self):
        before = PREPARE_MODULE.gps_stamp(2032, 604799.9)
        after = PREPARE_MODULE.gps_stamp(2033, 0.1)
        self.assertAlmostEqual(after - before, 0.2, places=5)


if __name__ == '__main__':
    unittest.main()
