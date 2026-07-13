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
"""Run reproducible UrbanNav Tokyo localization ablation experiments."""

import argparse
import csv
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec
from importlib.util import spec_from_loader
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


PROFILE_NAMES = (
    'baseline', 'nhc', 'robust', 'full', 'wheel', 'wheel_auto',
    'wheel_nhc', 'wheel_nhc_fixed')
ESTIMATE_TOPIC = '/ekf_localization/current_pose'


def deep_merge(base, override):
    """Recursively merge dictionaries without modifying either input."""
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_yaml(path):
    """Read one YAML mapping."""
    with open(path, encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError('{} does not contain a YAML mapping'.format(path))
    return value


def prepare_configs(
        base_profile, profiles_directory, output_directory,
        profile_names=PROFILE_NAMES):
    """Write the exact merged parameter file used by every experiment."""
    base = load_yaml(base_profile)
    config_directory = output_directory / 'configs'
    config_directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name in profile_names:
        override = load_yaml(profiles_directory / ('research_' + name + '.yaml'))
        merged = deep_merge(base, override)
        path = config_directory / (name + '.yaml')
        path.write_text(yaml.safe_dump(merged, sort_keys=False), encoding='utf-8')
        paths[name] = path
    return paths


def load_script_module(name):
    """Load a sibling script in source and installed layouts."""
    directory = Path(__file__).resolve().parent
    source_path = directory / (name + '.py')
    path = source_path if source_path.exists() else directory / name
    loader = SourceFileLoader(name, str(path))
    spec = spec_from_loader(loader.name, loader)
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


def write_trajectory(path, samples):
    """Write estimator samples in the common evaluation CSV format."""
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(('stamp', 'x', 'y', 'z', 'yaw'))
        for sample in samples:
            writer.writerow((sample.stamp, sample.x, sample.y, sample.z, sample.yaw))


def stop_process(process, timeout=10.0):
    """Gracefully stop a subprocess group and force it down if necessary."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=timeout)


def start(command, log_path):
    """Start a command in its own process group with a persistent log."""
    log = open(log_path, 'w', encoding='utf-8')
    process = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True)
    process.experiment_log = log
    return process


def run_profile(name, config, input_bag, output_directory, topic, rate, startup_delay):
    """Replay one bag, record its estimate, and export that estimate to CSV."""
    run_directory = output_directory / name
    run_directory.mkdir()
    estimate_bag = run_directory / 'estimate_bag'
    launch_command = [
        'ros2', 'launch', 'kalman_filter_localization', 'ekf.launch.py',
        'ekf_param_dir:=' + str(config.resolve()),
    ]
    record_command = [
        'ros2', 'bag', 'record', '--output', str(estimate_bag), topic,
    ]
    play_command = [
        'ros2', 'bag', 'play', str(input_bag), '--rate', str(rate),
        '--delay', str(startup_delay),
    ]
    launch = start(launch_command, run_directory / 'localization.log')
    recorder = None
    try:
        time.sleep(startup_delay)
        if launch.poll() is not None:
            raise RuntimeError('{} localization exited before playback'.format(name))
        recorder = start(record_command, run_directory / 'record.log')
        time.sleep(startup_delay)
        playback = subprocess.run(
            play_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, check=False)
        (run_directory / 'playback.log').write_text(playback.stdout, encoding='utf-8')
        if playback.returncode:
            raise RuntimeError('{} bag playback failed'.format(name))
        time.sleep(1.0)
    finally:
        if recorder is not None:
            stop_process(recorder)
            recorder.experiment_log.close()
        stop_process(launch)
        launch.experiment_log.close()
    evaluator = load_script_module('evaluate_localization')
    samples = evaluator.read_bag(str(estimate_bag), topic)
    if not samples:
        raise RuntimeError('{} produced no estimate samples'.format(name))
    estimate_csv = run_directory / 'estimate.csv'
    write_trajectory(estimate_csv, samples)
    return estimate_csv, [launch_command, record_command, play_command]


def command_manifest(
        configs, input_bag, output_directory, topic, rate, startup_delay,
        profile_names=PROFILE_NAMES):
    """Build a machine-readable dry-run manifest."""
    runs = []
    for name in profile_names:
        run_directory = output_directory / name
        runs.append({
            'name': name,
            'config': str(configs[name].resolve()),
            'estimate_csv': str((run_directory / 'estimate.csv').resolve()),
            'commands': [
                ['ros2', 'launch', 'kalman_filter_localization', 'ekf.launch.py',
                 'ekf_param_dir:=' + str(configs[name].resolve())],
                ['ros2', 'bag', 'record', '--output',
                 str(run_directory / 'estimate_bag'), topic],
                ['ros2', 'bag', 'play', str(input_bag), '--rate', str(rate),
                 '--delay', str(startup_delay)],
            ],
        })
    return {'input_bag': str(input_bag), 'estimate_topic': topic, 'runs': runs}


def main(argv=None):
    """Run all UrbanNav ablations and produce comparison artifacts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-bag', type=Path, required=True)
    parser.add_argument('--reference-csv', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--base-profile', type=Path, required=True)
    parser.add_argument('--profiles-dir', type=Path, required=True)
    parser.add_argument('--estimate-topic', default=ESTIMATE_TOPIC)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--startup-delay', type=float, default=2.0)
    parser.add_argument('--max-reference-gap', type=float, default=0.2)
    parser.add_argument('--time-offset', type=float, default=0.0)
    parser.add_argument(
        '--profile', action='append', choices=PROFILE_NAMES,
        help='run only this profile; repeat for multiple profiles')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise ValueError('output directory must be absent or empty')
        if not args.input_bag.exists():
            raise ValueError('input bag does not exist: {}'.format(args.input_bag))
        if not args.reference_csv.is_file():
            raise ValueError('reference CSV does not exist: {}'.format(args.reference_csv))
        if args.rate <= 0.0 or args.startup_delay < 0.0:
            raise ValueError('rate must be positive and startup delay non-negative')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        profile_names = tuple(args.profile) if args.profile else PROFILE_NAMES
        configs = prepare_configs(
            args.base_profile, args.profiles_dir, args.output_dir, profile_names)
        manifest = command_manifest(
            configs, args.input_bag, args.output_dir, args.estimate_topic,
            args.rate, args.startup_delay, profile_names)
        manifest.update({
            'reference_csv': str(args.reference_csv),
            'profiles': list(profile_names),
            'rate': args.rate,
            'startup_delay': args.startup_delay,
            'max_reference_gap': args.max_reference_gap,
            'time_offset': args.time_offset,
        })
        manifest_path = args.output_dir / 'manifest.json'
        manifest_path.write_text(
            json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
        if args.dry_run:
            print(manifest_path)
            return 0
        estimates = []
        for name in profile_names:
            estimate, unused_commands = run_profile(
                name, configs[name], args.input_bag, args.output_dir,
                args.estimate_topic, args.rate, args.startup_delay)
            estimates.append((name, estimate))
        comparator = load_script_module('compare_localization_results')
        rows = comparator.compare(
            args.reference_csv, estimates, args.max_reference_gap, args.time_offset)
        comparator.write_csv(args.output_dir / 'comparison.csv', rows)
        comparator.write_markdown(args.output_dir / 'comparison.md', rows)
        print(args.output_dir / 'comparison.md')
        return 0
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
