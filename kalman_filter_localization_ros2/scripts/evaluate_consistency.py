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
"""Summarize NEES/NIS samples and chi-square 95% coverage from a simulation CSV."""

import argparse
import csv
import json
import math
from statistics import mean, NormalDist
import sys


def chi_square_quantile(probability, degrees_of_freedom):
    """Wilson-Hilferty approximation; deterministic and dependency-free."""
    if not 0.0 < probability < 1.0 or degrees_of_freedom <= 0:
        raise ValueError('invalid chi-square arguments')
    z_value = NormalDist().inv_cdf(probability)
    dof = float(degrees_of_freedom)
    return max(0.0, dof * (
        1.0 - 2.0 / (9.0 * dof) + z_value * math.sqrt(2.0 / (9.0 * dof))) ** 3)


def summarize(samples, degrees_of_freedom):
    finite = [value for value in samples if math.isfinite(value) and value >= 0.0]
    if not finite:
        raise ValueError('consistency series is empty')
    lower = chi_square_quantile(0.025, degrees_of_freedom)
    upper = chi_square_quantile(0.975, degrees_of_freedom)
    average = mean(finite)
    mean_half_width = 1.96 * math.sqrt(2.0 * degrees_of_freedom / len(finite))
    return {
        'samples': len(finite), 'degrees_of_freedom': degrees_of_freedom,
        'mean': average, 'expected_mean': float(degrees_of_freedom),
        'mean_95_interval': [degrees_of_freedom - mean_half_width,
                             degrees_of_freedom + mean_half_width],
        'sample_95_interval': [lower, upper],
        'coverage_95': sum(lower <= value <= upper for value in finite) / len(finite),
    }


def evaluate_csv(path, specifications):
    columns = {name: [] for name, unused_dof in specifications}
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        missing = set(columns).difference(reader.fieldnames or [])
        if missing:
            raise ValueError('missing consistency columns: {}'.format(', '.join(sorted(missing))))
        for row in reader:
            for name in columns:
                if row[name] not in ('', None):
                    columns[name].append(float(row[name]))
    return {name: summarize(columns[name], dof) for name, dof in specifications}


def parse_series(value):
    fields = value.split(':')
    if len(fields) != 2:
        raise argparse.ArgumentTypeError('series must be COLUMN:DOF')
    try:
        dof = int(fields[1])
    except ValueError as error:
        raise argparse.ArgumentTypeError('DOF must be an integer') from error
    if dof <= 0:
        raise argparse.ArgumentTypeError('DOF must be positive')
    return fields[0], dof


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-csv', required=True)
    parser.add_argument('--series', action='append', type=parse_series, required=True)
    parser.add_argument('--output-json')
    args = parser.parse_args(argv)
    try:
        result = {'series': evaluate_csv(args.input_csv, args.series)}
        rendered = json.dumps(result, indent=2, sort_keys=True)
        print(rendered)
        if args.output_json:
            with open(args.output_json, 'w', encoding='utf-8') as stream:
                stream.write(rendered + '\n')
        return 0
    except (OSError, ValueError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
