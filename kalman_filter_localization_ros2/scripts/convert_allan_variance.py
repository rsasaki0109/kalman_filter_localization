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

"""Convert unit-tagged Allan-deviation coefficients into EKF noise YAML."""

import argparse
import json
import math
from pathlib import Path

import yaml


EXPECTED_UNITS = {
    'gyro_white_noise_density': 'rad/s/sqrt(Hz)',
    'accel_white_noise_density': 'm/s^2/sqrt(Hz)',
    'gyro_bias_random_walk': 'rad/s^2/sqrt(Hz)',
    'accel_bias_random_walk': 'm/s^3/sqrt(Hz)',
}


def convert(document):
    """Validate Allan coefficients and return continuous-time EKF variances."""
    values = {}
    for name, expected_unit in EXPECTED_UNITS.items():
        entry = document.get(name)
        if not isinstance(entry, dict) or entry.get('unit') != expected_unit:
            raise ValueError('{} must use unit {}'.format(name, expected_unit))
        value = float(entry.get('value', float('nan')))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError('{} value must be finite and nonnegative'.format(name))
        values[name] = value
    return {
        'var_imu_w': values['gyro_white_noise_density'] ** 2,
        'var_imu_acc': values['accel_white_noise_density'] ** 2,
        'var_imu_gyro_bias': values['gyro_bias_random_walk'] ** 2,
        'var_imu_acc_bias': values['accel_bias_random_walk'] ** 2,
    }


def main():
    """Run the command-line converter."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-json', required=True)
    parser.add_argument('--output-yaml', required=True)
    parser.add_argument('--output-report', required=True)
    args = parser.parse_args()

    input_path = Path(args.input_json)
    with input_path.open(encoding='utf-8') as stream:
        document = json.load(stream)
    parameters = convert(document)
    output = {'ekf_localization': {'ros__parameters': parameters}}
    with Path(args.output_yaml).open('w', encoding='utf-8') as stream:
        yaml.safe_dump(output, stream, sort_keys=True)

    lines = [
        '# Allan variance conversion report',
        '',
        'Input: `{}`'.format(input_path),
        '',
        '| Input coefficient | Unit | Value | EKF variance |',
        '|---|---|---:|---:|',
    ]
    mapping = (
        ('gyro_white_noise_density', 'var_imu_w'),
        ('accel_white_noise_density', 'var_imu_acc'),
        ('gyro_bias_random_walk', 'var_imu_gyro_bias'),
        ('accel_bias_random_walk', 'var_imu_acc_bias'),
    )
    for input_name, output_name in mapping:
        lines.append('| {} | {} | {:.12g} | `{}={:.12g}` |'.format(
            input_name, EXPECTED_UNITS[input_name], document[input_name]['value'],
            output_name, parameters[output_name]))
    lines.extend([
        '',
        'The EKF parameters are continuous-time power spectral densities: each value is the',
        'square of the corresponding amplitude density. No sampling-rate factor is applied here.',
        '',
    ])
    Path(args.output_report).write_text('\n'.join(lines), encoding='utf-8')


if __name__ == '__main__':
    main()
