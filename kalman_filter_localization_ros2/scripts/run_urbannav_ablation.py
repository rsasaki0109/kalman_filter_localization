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
import hashlib
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec
from importlib.util import spec_from_loader
import json
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
import time

import yaml


PROFILE_NAMES = (
    'baseline', 'nhc', 'robust', 'full', 'exact', 'replay', 'initialization',
    'initialization_wide', 'initialization_1s', 'wheel', 'wheel_auto',
    'wheel_nhc', 'wheel_nhc_fixed', 'wheel_full_cross', 'wheel_decoupled',
    'observability')
ESTIMATE_TOPIC = '/ekf_localization/current_pose'
DRAIN_SERVICE = '/ekf_localization/drain_input_buffer'


def parse_segment(value):
    """Parse a manifest segment as NAME,START,END."""
    fields = value.split(',')
    if len(fields) != 3:
        raise argparse.ArgumentTypeError('segment must be NAME,START,END')
    try:
        start, end = float(fields[1]), float(fields[2])
    except ValueError as error:
        raise argparse.ArgumentTypeError('segment timestamps must be numeric') from error
    if start >= end:
        raise argparse.ArgumentTypeError('segment START must be below END')
    return {'name': fields[0], 'start': start, 'end': end}


def sha256_path(path):
    """Hash one file or a directory tree, including relative file names."""
    path = Path(path)
    digest = hashlib.sha256()
    if path.is_file():
        with open(path, 'rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        return digest.hexdigest()
    if not path.is_dir():
        raise ValueError('cannot hash missing path: {}'.format(path))
    for child in sorted(item for item in path.rglob('*') if item.is_file()):
        relative = child.relative_to(path).as_posix().encode('utf-8')
        digest.update(len(relative).to_bytes(8, 'big'))
        digest.update(relative)
        digest.update(child.stat().st_size.to_bytes(8, 'big'))
        with open(child, 'rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    return digest.hexdigest()


def artifact_record(path):
    """Describe one immutable experiment input or generated configuration."""
    path = Path(path)
    return {
        'path': str(path.resolve()),
        'kind': 'directory' if path.is_dir() else 'file',
        'sha256': sha256_path(path),
    }


def runtime_artifacts():
    """Hash the installed estimator artifacts that ROS will actually execute."""
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'kalman_filter_localization'],
        check=True, capture_output=True, text=True)
    prefix = Path(result.stdout.strip())
    artifacts = {
        'node': prefix / 'lib' / 'kalman_filter_localization' /
        'ekf_localization_node',
        'component': prefix / 'lib' / 'libekf_localization_component.so',
        'runner': Path(__file__).resolve(),
    }
    records = {}
    for name, path in artifacts.items():
        if not path.is_file():
            raise ValueError('missing installed runtime artifact: {}'.format(path))
        record = artifact_record(path)
        record['invoked_path'] = str(path)
        records[name] = record
    return records


def rosbag_record(path):
    """Describe a rosbag and expose recorded topic counts when available."""
    record = artifact_record(path)
    metadata_path = Path(path) / 'metadata.yaml'
    if not metadata_path.is_file():
        return record
    try:
        metadata = load_yaml(metadata_path).get('rosbag2_bagfile_information', {})
    except (OSError, ValueError, yaml.YAMLError):
        return record
    topics = {}
    for item in metadata.get('topics_with_message_count', []):
        topic = item.get('topic_metadata', {}).get('name')
        count = item.get('message_count')
        if topic is not None and count is not None:
            topics[topic] = count
    record['rosbag'] = {
        'storage_identifier': metadata.get('storage_identifier'),
        'duration_nanoseconds': metadata.get('duration', {}).get('nanoseconds'),
        'starting_time_nanoseconds': metadata.get('starting_time', {}).get(
            'nanoseconds_since_epoch'),
        'message_count': metadata.get('message_count'),
        'topics': topics,
    }
    return record


def write_manifest(path, manifest):
    """Atomically persist progress so an interrupted experiment is auditable."""
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)


def read_component_counts(path):
    """Read the final machine-parseable input count line from a node log."""
    text = Path(path).read_text(encoding='utf-8')
    matches = re.findall(
        r'input_counts initial_pose=(\d+) imu=(\d+) odom=(\d+) '
        r'gnss_pose=(\d+) gnss_navsatfix=(\d+) gnss_doppler=(\d+) '
        r'wheel=(\d+) published_pose=(\d+) reorder_late=(\d+) '
        r'reorder_buffered=(\d+)', text)
    if not matches:
        return None
    names = (
        'initial_pose', 'imu', 'odom', 'gnss_pose', 'gnss_navsatfix',
        'gnss_doppler', 'wheel', 'published_pose', 'reorder_late',
        'reorder_buffered')
    return dict(zip(names, (int(value) for value in matches[-1])))


def read_performance_metrics(path):
    """Read the final machine-parseable performance line from a node log."""
    text = Path(path).read_text(encoding='utf-8')
    matches = re.findall(
        r'performance rtf=([0-9.eE+-]+) imu_callback_mean_us=([0-9.eE+-]+) '
        r'imu_callback_max_us=([0-9.eE+-]+) history_memory_bytes=(\d+) '
        r'history_imu=(\d+) history_measurements=(\d+) rewinds=(\d+)', text)
    if not matches:
        return None
    values = matches[-1]
    return {
        'real_time_factor': float(values[0]),
        'imu_callback_mean_us': float(values[1]),
        'imu_callback_max_us': float(values[2]),
        'history_memory_bytes': int(values[3]),
        'history_imu_samples': int(values[4]),
        'history_measurements': int(values[5]),
        'rewind_count': int(values[6]),
    }


def expected_component_counts(config_path, input_bag_record):
    """Map configured subscriptions to their expected rosbag topic counts."""
    parameters = load_yaml(config_path).get('ekf_localization', {}).get(
        'ros__parameters', {})
    topics = input_bag_record.get('rosbag', {}).get('topics', {})
    expected = {
        'initial_pose': topics.get(
            parameters.get('initial_pose_topic', '/ekf_localization/initial_pose'), 0),
        'imu': topics.get(parameters.get('imu_topic', '/ekf_localization/imu'), 0),
    }
    if parameters.get('use_odom', False):
        expected['odom'] = topics.get(parameters.get('odom_topic', '/ekf_localization/odom'), 0)
    if parameters.get('use_gnss', True):
        if parameters.get('gnss_input_type', 'pose') == 'navsatfix':
            expected['gnss_navsatfix'] = topics.get(
                parameters.get('gnss_navsatfix_topic', '/ekf_localization/gnss/fix'), 0)
        else:
            expected['gnss_pose'] = topics.get(
                parameters.get('gnss_pose_topic', '/ekf_localization/gnss_pose'), 0)
    if parameters.get('use_gnss_doppler_velocity', False):
        expected['gnss_doppler'] = topics.get(
            parameters.get(
                'gnss_doppler_velocity_topic', '/ekf_localization/gnss/velocity'), 0)
    if parameters.get('use_wheel_speed', False):
        expected['wheel'] = topics.get(parameters.get('wheel_speed_topic', '/wheel_speed'), 0)
    if parameters.get('output_publish_mode', 'timer') == 'imu':
        expected['published_pose'] = expected['imu']
    if parameters.get('input_reorder_window_sec', 0.0) > 0.0:
        expected['reorder_late'] = 0
        expected['reorder_buffered'] = 0
    return expected


def check_component_counts(expected, actual):
    """Compare expected and received inputs without ignoring surplus messages."""
    if actual is None:
        return {'passed': False, 'expected': expected, 'actual': None, 'mismatches': expected}
    mismatches = {
        name: {'expected': count, 'actual': actual.get(name, 0)}
        for name, count in expected.items() if actual.get(name, 0) != count
    }
    return {
        'passed': not mismatches,
        'expected': expected,
        'actual': {name: actual.get(name, 0) for name in expected},
        'mismatches': mismatches,
    }


def run_metadata_command(command, cwd=None):
    """Run a metadata probe without making manifest generation fragile."""
    try:
        result = subprocess.run(
            command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, check=False, timeout=5.0)
    except (OSError, subprocess.TimeoutExpired):
        return None
    value = result.stdout.rstrip()
    return value if result.returncode == 0 and value else None


def run_metadata_bytes(command, cwd=None):
    """Run a metadata probe and preserve its exact byte output."""
    try:
        result = subprocess.run(
            command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            check=False, timeout=5.0)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout if result.returncode == 0 else None


def repository_record(start_directory):
    """Capture the exact Git revision and working-tree fingerprint."""
    root = run_metadata_command(
        ['git', 'rev-parse', '--show-toplevel'], cwd=start_directory)
    if root is None:
        return {'available': False}
    commit = run_metadata_command(['git', 'rev-parse', 'HEAD'], cwd=root)
    status = run_metadata_command(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'], cwd=root)
    diff = run_metadata_bytes(['git', 'diff', '--binary', 'HEAD'], cwd=root)
    untracked_output = run_metadata_bytes(
        ['git', 'ls-files', '--others', '--exclude-standard', '-z'], cwd=root)
    status_lines = status.splitlines() if status else []
    untracked = {}
    if untracked_output:
        for raw_path in sorted(value for value in untracked_output.split(b'\0') if value):
            relative = raw_path.decode('utf-8', errors='surrogateescape')
            candidate = Path(root) / relative
            if candidate.is_file() or candidate.is_dir():
                untracked[relative] = sha256_path(candidate)
    fingerprint = hashlib.sha256()
    fingerprint.update(diff or b'')
    for relative, checksum in untracked.items():
        fingerprint.update(relative.encode('utf-8', errors='surrogateescape'))
        fingerprint.update(checksum.encode('ascii'))
    return {
        'available': True,
        'root': str(Path(root).resolve()),
        'commit': commit,
        'dirty': bool(status_lines),
        'status': status_lines,
        'tracked_diff_sha256': hashlib.sha256(diff or b'').hexdigest(),
        'untracked_artifacts': untracked,
        'working_tree_sha256': fingerprint.hexdigest(),
    }


def ros_package_version(name):
    """Return a ROS package version when the ROS CLI is available."""
    return run_metadata_command(['ros2', 'pkg', 'xml', '-t', 'version', name])


def environment_record():
    """Capture runtime versions that can affect replay and estimator output."""
    ros_packages = {}
    for name in ('rclcpp', 'rosbag2', 'geometry_msgs', 'sensor_msgs'):
        version = ros_package_version(name)
        if version is not None:
            ros_packages[name] = version
    return {
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'ros_version': os.environ.get('ROS_VERSION'),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
        'python': platform.python_version(),
        'python_executable': str(Path(sys.executable).resolve()),
        'platform': platform.platform(),
        'ros_packages': ros_packages,
    }


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


def run_profile(
        name, config, input_bag, output_directory, topic, rate, startup_delay,
        drain_timeout_sec=300.0):
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
        drain_command = [
            'ros2', 'service', 'call', DRAIN_SERVICE, 'std_srvs/srv/Trigger', '{}']
        try:
            drain = subprocess.run(
                drain_command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, check=False, timeout=drain_timeout_sec)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError(
                '{} input-buffer drain exceeded {:.1f} sec'.format(
                    name, drain_timeout_sec)) from error
        (run_directory / 'drain.log').write_text(drain.stdout, encoding='utf-8')
        drain_succeeded = re.search(
            r'success\s*(?:=|:)\s*(?:True|true)', drain.stdout) is not None
        if drain.returncode or not drain_succeeded:
            raise RuntimeError('{} input-buffer drain failed'.format(name))
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
    component_counts = read_component_counts(run_directory / 'localization.log')
    performance = read_performance_metrics(run_directory / 'localization.log')
    return estimate_csv, {
        'completed': True,
        'estimate_samples': len(samples),
        'estimate_csv': artifact_record(estimate_csv),
        'estimate_bag': rosbag_record(estimate_bag),
        'component_counts': component_counts,
        'performance': performance,
        'logs': {
            name: artifact_record(run_directory / (name + '.log'))
            for name in ('localization', 'record', 'playback', 'drain')
        },
    }


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
            'config_artifact': artifact_record(configs[name]),
            'estimate_csv': str((run_directory / 'estimate.csv').resolve()),
            'commands': [
                ['ros2', 'launch', 'kalman_filter_localization', 'ekf.launch.py',
                 'ekf_param_dir:=' + str(configs[name].resolve())],
                ['ros2', 'bag', 'record', '--output',
                 str(run_directory / 'estimate_bag'), topic],
                ['ros2', 'bag', 'play', str(input_bag), '--rate', str(rate),
                 '--delay', str(startup_delay)],
                ['ros2', 'service', 'call', DRAIN_SERVICE,
                 'std_srvs/srv/Trigger', '{}'],
            ],
        })
    return {
        'schema_version': 2,
        'input_bag': str(input_bag.resolve()),
        'estimate_topic': topic,
        'runs': runs,
    }


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
    parser.add_argument('--drain-timeout-sec', type=float, default=300.0)
    parser.add_argument('--max-reference-gap', type=float, default=0.2)
    parser.add_argument('--time-offset', type=float, default=0.0)
    parser.add_argument('--align-translation', action='store_true')
    parser.add_argument('--segment', type=parse_segment, action='append', default=[])
    parser.add_argument('--split-role', choices=('tuning', 'holdout'), default='tuning')
    parser.add_argument(
        '--profile', action='append', choices=PROFILE_NAMES,
        help='run only this profile; repeat for multiple profiles')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument(
        '--require-complete-input', action='store_true',
        help='fail a run when component input counts do not match rosbag metadata')
    args = parser.parse_args(argv)
    try:
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise ValueError('output directory must be absent or empty')
        if not args.input_bag.exists():
            raise ValueError('input bag does not exist: {}'.format(args.input_bag))
        if not args.reference_csv.is_file():
            raise ValueError('reference CSV does not exist: {}'.format(args.reference_csv))
        if args.rate <= 0.0 or args.startup_delay < 0.0 or args.drain_timeout_sec <= 0.0:
            raise ValueError(
                'rate and drain timeout must be positive; startup delay must be non-negative')
        args.output_dir.mkdir(parents=True, exist_ok=True)
        profile_names = tuple(args.profile) if args.profile else PROFILE_NAMES
        if args.split_role == 'holdout' and len(profile_names) != 1:
            raise ValueError('holdout evaluation requires exactly one final profile')
        configs = prepare_configs(
            args.base_profile, args.profiles_dir, args.output_dir, profile_names)
        manifest = command_manifest(
            configs, args.input_bag, args.output_dir, args.estimate_topic,
            args.rate, args.startup_delay, profile_names)
        manifest.update({
            'reference_csv': str(args.reference_csv.resolve()),
            'profiles': list(profile_names),
            'rate': args.rate,
            'startup_delay': args.startup_delay,
            'drain_timeout_sec': args.drain_timeout_sec,
            'max_reference_gap': args.max_reference_gap,
            'time_offset': args.time_offset,
            'evaluation_policy': {
                'alignment': 'median_translation' if args.align_translation else 'none',
                'interpolation_tolerance_sec': args.max_reference_gap,
                'time_offset_sec': args.time_offset,
                'segments': args.segment,
                'split_role': args.split_role,
            },
            'require_complete_input': args.require_complete_input,
            'repository': repository_record(Path(__file__).resolve().parent),
            'environment': environment_record(),
            'runtime_artifacts': runtime_artifacts(),
            'inputs': {
                'bag': rosbag_record(args.input_bag),
                'reference': artifact_record(args.reference_csv),
                'base_profile': artifact_record(args.base_profile),
                'profile_overrides': {
                    name: artifact_record(
                        args.profiles_dir / ('research_' + name + '.yaml'))
                    for name in profile_names
                },
            },
        })
        manifest_path = args.output_dir / 'manifest.json'
        write_manifest(manifest_path, manifest)
        if args.dry_run:
            print(manifest_path)
            return 0
        estimates = []
        input_bag_record = manifest['inputs']['bag']
        for name in profile_names:
            estimate, result = run_profile(
                name, configs[name], args.input_bag, args.output_dir,
                args.estimate_topic, args.rate, args.startup_delay,
                args.drain_timeout_sec)
            estimates.append((name, estimate))
            expected_counts = expected_component_counts(configs[name], input_bag_record)
            result['input_count_check'] = check_component_counts(
                expected_counts, result['component_counts'])
            next(run for run in manifest['runs'] if run['name'] == name)['result'] = result
            write_manifest(manifest_path, manifest)
            if args.require_complete_input and not result['input_count_check']['passed']:
                raise RuntimeError(
                    '{} input count mismatch: {}'.format(
                        name, result['input_count_check']['mismatches']))
        comparator = load_script_module('compare_localization_results')
        rows = comparator.compare(
            args.reference_csv, estimates, args.max_reference_gap, args.time_offset,
            args.align_translation)
        comparator.write_csv(args.output_dir / 'comparison.csv', rows)
        comparator.write_markdown(args.output_dir / 'comparison.md', rows)
        manifest['completed'] = True
        manifest['outputs'] = {
            'comparison_csv': artifact_record(args.output_dir / 'comparison.csv'),
            'comparison_markdown': artifact_record(args.output_dir / 'comparison.md'),
        }
        write_manifest(manifest_path, manifest)
        print(args.output_dir / 'comparison.md')
        return 0
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print('error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
