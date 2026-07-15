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
REPEATABILITY_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'verify_repeatability.py')
PREPARE_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'prepare_urbannav_tokyo.py')
APPLANIX_PREPARE_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'prepare_applanix_open_sky.py')
ALLAN_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'convert_allan_variance.py')
CONSISTENCY_SCRIPT = (
    Path(__file__).parents[1] / 'scripts' / 'evaluate_consistency.py')
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
REPEATABILITY_SPEC = importlib.util.spec_from_file_location(
    'verify_repeatability', REPEATABILITY_SCRIPT)
REPEATABILITY_MODULE = importlib.util.module_from_spec(REPEATABILITY_SPEC)
REPEATABILITY_SPEC.loader.exec_module(REPEATABILITY_MODULE)
ALLAN_SPEC = importlib.util.spec_from_file_location(
    'convert_allan_variance', ALLAN_SCRIPT)
ALLAN_MODULE = importlib.util.module_from_spec(ALLAN_SPEC)
ALLAN_SPEC.loader.exec_module(ALLAN_MODULE)
CONSISTENCY_SPEC = importlib.util.spec_from_file_location(
    'evaluate_consistency', CONSISTENCY_SCRIPT)
CONSISTENCY_MODULE = importlib.util.module_from_spec(CONSISTENCY_SPEC)
CONSISTENCY_SPEC.loader.exec_module(CONSISTENCY_MODULE)
PREPARE_SPEC = importlib.util.spec_from_file_location(
    'prepare_urbannav_tokyo', PREPARE_SCRIPT)
PREPARE_MODULE = importlib.util.module_from_spec(PREPARE_SPEC)
PREPARE_SPEC.loader.exec_module(PREPARE_MODULE)
APPLANIX_PREPARE_SPEC = importlib.util.spec_from_file_location(
    'prepare_applanix_open_sky', APPLANIX_PREPARE_SCRIPT)
APPLANIX_PREPARE_MODULE = importlib.util.module_from_spec(APPLANIX_PREPARE_SPEC)
APPLANIX_PREPARE_SPEC.loader.exec_module(APPLANIX_PREPARE_MODULE)


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

    def test_translation_alignment_removes_constant_datum_offset(self):
        references = [MODULE.Sample(0.0, 0.0, 0.0, 0.0, 0.0),
                      MODULE.Sample(1.0, 1.0, 2.0, 3.0, 0.0)]
        estimates = [MODULE.Sample(0.0, 4.0, -5.0, 8.0, 0.0),
                     MODULE.Sample(1.0, 5.0, -3.0, 11.0, 0.0)]
        summary, errors = MODULE.evaluate(
            estimates, references, 2.0, align_translation=True)
        self.assertAlmostEqual(summary['rmse_3d_m'], 0.0)
        self.assertEqual(
            summary['translation_alignment_m'], {'x': 4.0, 'y': -5.0, 'z': 8.0})
        self.assertTrue(all(row['error_3d'] == 0.0 for row in errors))

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

    def test_evaluate_selects_inclusive_time_range(self):
        estimates = [MODULE.Sample(stamp, stamp, 0.0, 0.0, 0.0)
                     for stamp in (0.0, 1.0, 2.0)]
        references = list(estimates)
        summary, errors = MODULE.evaluate(
            estimates, references, 0.2, start_stamp=1.0, end_stamp=2.0)
        self.assertEqual(summary['estimate_samples'], 2)
        self.assertEqual([row['stamp'] for row in errors], [1.0, 2.0])

    def test_evaluate_rejects_reversed_time_range(self):
        samples = [MODULE.Sample(1.0, 0.0, 0.0, 0.0, 0.0)]
        with self.assertRaisesRegex(ValueError, 'start-stamp'):
            MODULE.evaluate(samples, samples, 0.2, start_stamp=2.0, end_stamp=1.0)

    def test_evaluate_reports_ape_rpe_missing_policy_and_outage(self):
        references = [MODULE.Sample(float(index), float(index), 0.0, 0.0, 0.0)
                      for index in range(6)]
        estimates = [MODULE.Sample(float(index), 1.1 * index, 0.0, 0.0, 0.0)
                     for index in range(6)]
        summary, unused_errors = MODULE.evaluate(
            estimates, references, 0.25,
            rpe_time_horizons=(1.0,), rpe_distance_horizons=(2.0,),
            outage_segments=({'name': 'test', 'start': 1.0, 'end': 3.0},),
            settling_threshold_m=0.25, settling_window_sec=1.0)
        self.assertAlmostEqual(
            summary['ape']['translation_3d_m']['rmse'], summary['rmse_3d_m'])
        self.assertAlmostEqual(
            summary['rpe']['time']['1.0']['translation_m']['rmse'], 0.1)
        self.assertAlmostEqual(
            summary['rpe']['distance']['2.0']['translation_m']['rmse'], 0.2)
        self.assertEqual(summary['missing_ratio'], 0.0)
        self.assertEqual(summary['evaluation_policy']['alignment'], 'none')
        self.assertEqual(summary['outages'][0]['name'], 'test')
        self.assertAlmostEqual(summary['outages'][0]['endpoint_error_3d_m'], 0.3)

    def test_parse_outage_segment_validates_order(self):
        self.assertEqual(MODULE.parse_outage_segment('urban,1,2'),
                         {'name': 'urban', 'start': 1.0, 'end': 2.0})
        with self.assertRaisesRegex(Exception, 'below END'):
            MODULE.parse_outage_segment('bad,2,1')

    def test_consistency_evaluator_reports_nees_nis_and_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'consistency.csv'
            path.write_text(
                'nees,nis_position\n15,3\n14,2\n16,4\n15,3\n', encoding='utf-8')
            result = CONSISTENCY_MODULE.evaluate_csv(
                path, [('nees', 15), ('nis_position', 3)])
        self.assertEqual(result['nees']['mean'], 15.0)
        self.assertEqual(result['nis_position']['mean'], 3.0)
        self.assertEqual(result['nees']['coverage_95'], 1.0)
        self.assertEqual(result['nis_position']['coverage_95'], 1.0)

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
            with mock.patch.object(
                    RUNNER_MODULE, 'runtime_artifacts', return_value={
                        'node': {'path': '/installed/node', 'sha256': '1' * 64},
                        'component': {
                            'path': '/installed/component.so', 'sha256': '2' * 64},
                        'runner': {'path': '/installed/runner', 'sha256': '3' * 64},
                    }):
                result = RUNNER_MODULE.main([
                    '--input-bag', str(input_bag),
                    '--reference-csv', str(DATA / 'reference.csv'),
                    '--output-dir', str(output),
                    '--base-profile', str(profiles / 'urbannav_tokyo_tuned.yaml'),
                    '--profiles-dir', str(profiles),
                    '--profile', 'wheel',
                    '--dry-run',
                ])
            self.assertEqual(result, 0)
            manifest = json.loads(
                (output / 'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(manifest['schema_version'], 2)
            self.assertEqual([run['name'] for run in manifest['runs']], ['wheel'])
            self.assertEqual(len(manifest['inputs']['bag']['sha256']), 64)
            self.assertEqual(len(manifest['inputs']['reference']['sha256']), 64)
            self.assertEqual(len(manifest['runs'][0]['config_artifact']['sha256']), 64)
            self.assertIn('commit', manifest['repository'])
            self.assertIn('working_tree_sha256', manifest['repository'])
            self.assertIn('python', manifest['environment'])
            self.assertEqual(
                manifest['runtime_artifacts']['component']['sha256'], '2' * 64)
            self.assertEqual(manifest['evaluation_policy']['alignment'], 'none')
            self.assertEqual(manifest['evaluation_policy']['split_role'], 'tuning')
            self.assertEqual(manifest['evaluation_policy']['segments'], [])
            self.assertEqual(manifest['drain_timeout_sec'], 300.0)
            wheel = yaml.safe_load(
                (output / 'configs' / 'wheel.yaml').read_text(encoding='utf-8'))
            parameters = wheel['ekf_localization']['ros__parameters']
            self.assertTrue(parameters['use_gnss'])
            self.assertTrue(parameters['use_wheel_speed'])

    def test_parameter_profiles_only_override_declared_base_parameters(self):
        parameter_root = Path(__file__).parents[1] / 'param'
        base = yaml.safe_load((parameter_root / 'ekf.yaml').read_text(encoding='utf-8'))
        declared = set(base['ekf_localization']['ros__parameters'])
        for profile_path in (parameter_root / 'profiles').glob('research_*.yaml'):
            profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
            node = profile.get('ekf_localization', profile.get('/**', {}))
            overrides = set(node.get('ros__parameters', {}))
            self.assertFalse(overrides - declared,
                             '{} has undeclared parameters {}'.format(
                                 profile_path.name, sorted(overrides - declared)))

    def test_urbannav_runner_directory_hash_includes_names_and_contents(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'a').write_bytes(b'one')
            first = RUNNER_MODULE.sha256_path(root)
            self.assertEqual(first, RUNNER_MODULE.sha256_path(root))
            (root / 'a').rename(root / 'b')
            self.assertNotEqual(first, RUNNER_MODULE.sha256_path(root))
            renamed = RUNNER_MODULE.sha256_path(root)
            (root / 'b').write_bytes(b'two')
            self.assertNotEqual(renamed, RUNNER_MODULE.sha256_path(root))

    def test_urbannav_runner_hashes_installed_runtime_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = Path(directory)
            executable = prefix / 'lib' / 'kalman_filter_localization' / \
                'ekf_localization_node'
            component = prefix / 'lib' / 'libekf_localization_component.so'
            executable.parent.mkdir(parents=True)
            component.parent.mkdir(parents=True, exist_ok=True)
            executable.write_bytes(b'node-binary')
            component.write_bytes(b'component-binary')
            completed = mock.Mock(stdout=str(prefix) + '\n')
            with mock.patch.object(
                    RUNNER_MODULE.subprocess, 'run', return_value=completed) as run:
                artifacts = RUNNER_MODULE.runtime_artifacts()
            run.assert_called_once_with(
                ['ros2', 'pkg', 'prefix', 'kalman_filter_localization'],
                check=True, capture_output=True, text=True)
            self.assertEqual(
                artifacts['node']['sha256'],
                RUNNER_MODULE.sha256_path(executable))
            self.assertEqual(
                artifacts['component']['sha256'],
                RUNNER_MODULE.sha256_path(component))
            self.assertEqual(
                artifacts['runner']['path'], str(RUNNER_SCRIPT.resolve()))

    def test_urbannav_runner_parses_component_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'localization.log'
            path.write_text(
                '[INFO] input_counts initial_pose=20 imu=62040 odom=0 '
                'gnss_pose=3790 gnss_navsatfix=0 gnss_doppler=0 '
                'wheel=62040 published_pose=62040 reorder_late=0 '
                'reorder_buffered=0\n', encoding='utf-8')
            counts = RUNNER_MODULE.read_component_counts(path)
            self.assertEqual(counts['imu'], 62040)
            self.assertEqual(counts['gnss_pose'], 3790)
            self.assertEqual(counts['published_pose'], 62040)
            self.assertEqual(counts['reorder_late'], 0)

    def test_urbannav_runner_parses_performance_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'localization.log'
            path.write_text(
                '[INFO] performance rtf=2.5 imu_callback_mean_us=125.0 '
                'imu_callback_max_us=800.0 history_memory_bytes=4096 '
                'history_imu=101 history_measurements=5 rewinds=3\n',
                encoding='utf-8')
            metrics = RUNNER_MODULE.read_performance_metrics(path)
        self.assertEqual(metrics['real_time_factor'], 2.5)
        self.assertEqual(metrics['history_memory_bytes'], 4096)
        self.assertEqual(metrics['rewind_count'], 3)

    def test_urbannav_runner_checks_configured_input_counts(self):
        profiles = Path(__file__).parents[1] / 'param' / 'profiles'
        bag = {'rosbag': {'topics': {
            '/ekf_localization/initial_pose': 20,
            '/sensing/imu/imu_data': 62040,
            '/gnss_pose': 3790,
            '/wheel_speed': 62040,
        }}}
        expected = RUNNER_MODULE.expected_component_counts(
            profiles / 'urbannav_tokyo_tuned.yaml', bag)
        self.assertEqual(expected['imu'], 62040)
        self.assertEqual(expected['gnss_pose'], 3790)
        self.assertEqual(expected['published_pose'], 62040)
        self.assertEqual(expected['reorder_late'], 0)
        passed = RUNNER_MODULE.check_component_counts(
            expected, dict(expected, published_pose=62040))
        self.assertTrue(passed['passed'])
        failed = RUNNER_MODULE.check_component_counts(
            expected, dict(expected, imu=60000))
        self.assertFalse(failed['passed'])
        self.assertEqual(failed['mismatches']['imu']['expected'], 62040)

    def test_repeatability_comparison_reports_first_numerical_divergence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / 'reference.csv'
            candidate = root / 'candidate.csv'
            reference.write_text(
                'stamp,x,y,z,yaw\n1,0,0,0,3.141592653589793\n2,1,2,3,0\n',
                encoding='utf-8')
            candidate.write_text(
                'stamp,x,y,z,yaw\n1,0,0,0,-3.141592653589793\n2,1.001,2,3,0\n',
                encoding='utf-8')
            report = REPEATABILITY_MODULE.compare_csv(
                reference, candidate, stamp_tolerance=0.0,
                value_tolerance=1.0e-6)
            self.assertFalse(report['passed'])
            self.assertEqual(report['first_mismatch']['sample_index'], 2)
            self.assertAlmostEqual(
                report['maximum_absolute_difference']['x'], 0.001)
            self.assertLess(report['maximum_absolute_difference']['yaw'], 1.0e-12)

    def test_repeatability_comparison_rejects_sample_count_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            reference = root / 'reference.csv'
            candidate = root / 'candidate.csv'
            reference.write_text(
                'stamp,x,y,z,yaw\n1,0,0,0,0\n2,0,0,0,0\n',
                encoding='utf-8')
            candidate.write_text(
                'stamp,x,y,z,yaw\n1,0,0,0,0\n', encoding='utf-8')
            report = REPEATABILITY_MODULE.compare_csv(
                reference, candidate, stamp_tolerance=0.0,
                value_tolerance=0.0)
            self.assertFalse(report['passed'])
            self.assertEqual(report['reference_samples'], 2)
            self.assertEqual(report['candidate_samples'], 1)
            self.assertEqual(report['first_mismatch']['reason'], 'sample_count')

    def test_urbannav_geodetic_origin_maps_to_zero_enu(self):
        origin = (35.62931853, 139.78712595, 44.6995)
        ecef = PREPARE_MODULE.geodetic_to_ecef(*origin)
        enu = PREPARE_MODULE.ecef_to_enu(
            ecef, ecef, origin[0], origin[1])
        self.assertEqual(enu, (0.0, 0.0, 0.0))

    def test_applanix_gravity_compensated_acceleration_restores_specific_force(self):
        acceleration = APPLANIX_PREPARE_MODULE.applanix_acceleration_to_specific_force(
            0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(acceleration[0], 0.0)
        self.assertAlmostEqual(acceleration[1], 0.0)
        self.assertAlmostEqual(acceleration[2], 9.80665)

    def test_applanix_specific_force_mode_only_changes_axes(self):
        acceleration = APPLANIX_PREPARE_MODULE.applanix_acceleration_to_specific_force(
            1.0, 2.0, 3.0, 10.0, 20.0, gravity_compensated=False)
        self.assertEqual(acceleration, (1.0, -2.0, -3.0))

    def test_urbannav_gps_stamp_is_continuous_across_weeks(self):
        before = PREPARE_MODULE.gps_stamp(2032, 604799.9)
        after = PREPARE_MODULE.gps_stamp(2033, 0.1)
        self.assertAlmostEqual(after - before, 0.2, places=5)

    def test_allan_converter_squares_unit_tagged_amplitude_densities(self):
        document = {
            'gyro_white_noise_density': {
                'value': 0.02, 'unit': 'rad/s/sqrt(Hz)'},
            'accel_white_noise_density': {
                'value': 0.1, 'unit': 'm/s^2/sqrt(Hz)'},
            'gyro_bias_random_walk': {
                'value': 0.003, 'unit': 'rad/s^2/sqrt(Hz)'},
            'accel_bias_random_walk': {
                'value': 0.004, 'unit': 'm/s^3/sqrt(Hz)'},
        }
        converted = ALLAN_MODULE.convert(document)
        self.assertAlmostEqual(converted['var_imu_w'], 0.0004)
        self.assertAlmostEqual(converted['var_imu_acc'], 0.01)
        self.assertAlmostEqual(converted['var_imu_gyro_bias'], 0.000009)
        self.assertAlmostEqual(converted['var_imu_acc_bias'], 0.000016)
        document['gyro_white_noise_density']['unit'] = 'deg/s/sqrt(Hz)'
        with self.assertRaises(ValueError):
            ALLAN_MODULE.convert(document)


if __name__ == '__main__':
    unittest.main()
