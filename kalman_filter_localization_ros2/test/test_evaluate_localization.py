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
import math
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'evaluate_localization.py'
DATA = Path(__file__).parent / 'data'
SPEC = importlib.util.spec_from_file_location('evaluate_localization', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


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


if __name__ == '__main__':
    unittest.main()
