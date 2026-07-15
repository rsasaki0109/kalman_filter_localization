# Phase 0 baseline protocol

This document freezes the inputs, replay policy, evaluation intervals, and
repeatability gate used before changing the estimator mathematics.

## Scope and data roles

The existing Odaiba and Applanix recordings have already been inspected and
tuned against. They are regression datasets, not scientific holdouts.

- Applanix open-sky: coordinate, gravity, timestamp, and normal-operation
  regression.
- UrbanNav Odaiba: urban continuous, outage, and reacquisition regression.
- Istanbul bag4--6: short-run initialization and bias-convergence regression.
- A previously unused UrbanNav sequence: final holdout. It must be registered
  here, including its checksum and intervals, before its reference trajectory
  is used for tuning.

No metric from a regression dataset may be described as holdout performance.

## Istanbul source acquisition

Istanbul `all-sensors-bag4_compressed` through `bag6` are the short Isuzu-bus
recordings published by Autoware. They are not currently stored in the
workspace. The authoritative source is:

```text
s3://autoware-files/recordings/bags/2022-08-22_leo_drive_isuzu_bags/
```

The current objects contain 17.84 s, 30.04 s, and 20.23 s respectively and
total approximately 2.9 GB. Download each directory with `aws s3 sync` and
`--no-sign-request`, then record object ETags and local SHA-256 values before
conversion. The converter's custom message dependency must be built from
`autowarefoundation/applanix` commit
`6689c1ba2796b2c6cc77a6e9c5dd1c1507d8f862` (or an explicitly recorded
replacement).

The source objects selected on 2026-07-15 are versioned as follows; a changed
version ID or ETag is a different Phase 0 input and must not silently replace
one of these objects.

| Bag | Compressed bytes | ETag | S3 version ID |
|---|---:|---|---|
| bag4 | 738,403,581 | `0b0f188ab41cc35a98254943ae5c1661` | `wABT6gXdTeL20soEyuRy3rXNMzLJo8zT` |
| bag5 | 1,272,164,992 | `1b8977f606ca7e1978be193bd80a36d7` | `Ol4ejDRQHihLE7dMXcs_qyS.dJmkybJn` |
| bag6 | 881,370,847 | `afd027b37c12e3562710b1be29ad7742` | `YeUdM2tI2x01HXFmbp7WE11gVmSz0WIw` |

The completed local downloads have these SHA-256 values:

| Bag | Compressed DB SHA-256 | Metadata SHA-256 |
|---|---|---|
| bag4 | `0cf7093c0635b931735b12dd0fb777ccca26ac5951d973076f5011ab8487884c` | `517857115440ae4dfd07a5fefa0876e247fcb02c95ed782a9160d1b0ea8935c6` |
| bag5 | `f6f3435b58191e4201104e19531f96285541e1ad82de4456586f7f307022c742` | `11b7e972289d4ce33e114fb5e03080dc0e2c713f483337de705f5d2b231f3aea` |
| bag6 | `5877539a1d34d083b7c89e65f1b0153bc0d23341057ea2b62c90c417f2635c20` | `3e57e4639baa4519e48c4ee74fb3c4675144c3cf562e276b55bf40e8bf9664fc` |

These three recordings contain no usable wheel-speed samples: bag4 and bag5
declare the topic with zero messages, and bag6 omits it. Their Phase 0 role is
therefore GNSS/IMU initialization and bias-convergence regression, not wheel
fusion evaluation.

After verifying the compressed-file SHA-256, expand each `*.db3.zstd`, source
ROS Jazzy and `.data/applanix_ws/install/setup.bash`, and convert it with:

```bash
python3 kalman_filter_localization_ros2/scripts/prepare_applanix_open_sky.py \
  --input-db3 <expanded.db3> \
  --output-bag .data/phase0/istanbul/bag<N>/input_bag \
  --output-reference-csv .data/phase0/istanbul/bag<N>/reference.csv \
  --gnss-rate-hz 5.0 --acceleration-mode gravity-compensated
```

Run `baseline` and `full` with
`istanbul_all_sensors_bag4_6.yaml` as the base profile, `--rate 10.0`, and
`--require-complete-input`. The base profile enables IMU-triggered output and
complete-bag deterministic drain; removing those settings invalidates the
schema-v2 Phase 0 result.

All three converted recordings passed that gate. The authoritative manifests
are `.data/phase0/istanbul/bag<N>/results/manifest.json`.

| Bag | IMU/output | GNSS | Profile | 3D RMSE m | Horizontal m | Yaw RMSE deg |
|---|---:|---:|---|---:|---:|---:|
| bag4 | 1,793 | 87 | baseline | 0.0983 | 0.0959 | 2.7106 |
| bag4 | 1,793 | 87 | full | 0.1562 | 0.0902 | 0.6711 |
| bag5 | 3,005 | 147 | baseline | 0.0967 | 0.0944 | 4.6611 |
| bag5 | 3,005 | 147 | full | 0.1346 | 0.0689 | 2.8273 |
| bag6 | 2,031 | 99 | baseline | 0.1168 | 0.1108 | 2.6286 |
| bag6 | 2,031 | 99 | full | 0.1651 | 0.1083 | 0.7042 |

Every row has match ratio 1.0, exact expected input/output counts, and zero
late or buffered events.

## Immutable Odaiba inputs

The Phase 0 input artifacts currently available in this workspace are:

| Artifact | SHA-256 |
|---|---|
| `odaiba_input_bag_wheel` directory | `20408e622c9aa227ae3778941a943dd8ab0051a014b0ff39e97d5d2c752bd4fb` |
| `odaiba_reference_wheel.csv` | `13ac1b1e99369f2fb27594c0316d3ffba194cd914ccf80b3015738fe4204ca07` |
| `urbannav_tokyo_tuned.yaml` | `56aa16ac359a36003aca38abdf933200060be2beef118108d03131529a895443` |

The bag contains 127,890 messages over 1,243 seconds:

| Topic | Expected messages |
|---|---:|
| `/sensing/imu/imu_data` | 62,040 |
| `/wheel_speed` | 62,040 |
| `/gnss_pose` | 3,790 |
| `/ekf_localization/initial_pose` | 20 |

Every experiment manifest must record these counts and checksums rather than
relying on file names.

## Replay policy

The accepted regression path decouples DDS delivery order from estimator order.
All subscribed inputs are first buffered, then the runner explicitly drains the
complete bag in `(sensor timestamp, fixed topic priority, per-topic sequence)`
order. The fixed priority for equal timestamps is initial pose, IMU, GNSS
position, GNSS Doppler velocity, wheel speed, then odometry. IMU-triggered
outputs are throttled during drain so the recorder does not lose the tail.

Because estimator ordering no longer depends on wall time, `--rate 10.0` is an
accepted transport rate only when all count, late-input, empty-buffer,
provenance, and sample-level gates pass. Ordinary online operation leaves the
reorder buffer disabled by default.

At 10x playback, two otherwise identical runs produced different output sample
counts and materially different outage metrics:

| Profile | Run 1 samples | Run 2 samples | Run 1 outage RMSE m | Run 2 outage RMSE m |
|---|---:|---:|---:|---:|
| baseline | 12,497 | 12,750 | 632.5852 | 653.4970 |
| wheel + NHC auto | 12,815 | 12,728 | 6.3473 | 7.1808 |

This fails the repeatability gate. Instrumentation then separated two failure
modes:

| Replay/QoS | IMU received | GNSS received | Outcome |
|---|---:|---:|---|
| 10x, depth 1 | 59,292 / 62,040 | 3,790 / 3,790 | input-loss gate failed |
| 1x, depth 1 | 62,015 / 62,040 | 3,790 / 3,790 | input-loss gate failed |
| 10x, depth 1000, run 1 | 62,040 / 62,040 | 3,790 / 3,790 | count gate passed |
| 10x, depth 1000, run 2 | 62,040 / 62,040 | 3,790 / 3,790 | count gate passed, trajectory gate failed |

The two lossless 10x runs both emitted 62,040 poses, but their estimate CSV
hashes differed and the first numerical divergence occurred at output sample 8.
Their full-trajectory 3D RMSE values were 196.8837 m and 200.9142 m. Therefore
deep QoS fixes input loss but does not make accelerated multi-topic replay
deterministic; changed callback ordering remains a separate failure. Neither a
wide metric tolerance nor matching message counts may hide it.

Two lossless 1x runs without deterministic buffering also diverged at output
sample 3,110 despite identical provenance. The maximum coordinate differences
reached 0.168/0.813/1.678 m and yaw differed by 0.0266 rad. Real-time playback
alone is therefore not an ordering guarantee.

Complete-bag deterministic buffering resolved both failure modes. Three 10x
runs processed the same 65,850 baseline events, each received every input,
published 62,040 poses, reported zero late and buffered events, and produced the
same estimate CSV SHA-256:
`fa67e137e9d89300d22231d29dde005211bc44cd448527f85eed1891e8942e82`.
Every timestamp and `x/y/z/yaw` value agreed exactly (maximum difference 0.0).

The `research_full` profile independently passed the same three-run gate. Each
run received 20 initial poses, 62,040 IMU samples, and 3,790 GNSS poses,
published 62,040 poses, and ended with zero late or buffered events. All three
estimate CSVs have SHA-256
`72fb39968182f8a02fd94279a375f5fdad744d03656de2d039d163c0d31299c5`;
the maximum per-sample difference in timestamp, `x/y/z`, and yaw was 0.0. The
machine-readable evidence is
`.data/phase0/odaiba_full_batch_10x_repeatability.json`.

The synchronized Applanix open-sky recording was also regenerated with a
schema-v2 manifest at `.data/phase0/applanix_batch_10x/manifest.json`. Both
profiles received 30 initial poses, 85,344 IMU samples, and 4,157 GNSS poses,
published 85,344 poses, and ended with zero late or buffered events. Baseline
3D/horizontal/yaw RMSE was 14.6091 m / 0.3056 m / 1.3207 deg; `research_full`
was 19.0266 m / 10.6587 m / 8.2281 deg. These numbers freeze current behaviour
and are not acceptance targets for the later estimator redesign.

Segment evaluation of the accepted Odaiba run is stored under
`.data/phase0/odaiba_segments`. With translation-only alignment, continuous
3D RMSE is 0.2470 m for `baseline` and 0.2233 m for `research_full`. Without
alignment, the 81.8 s outage 3D RMSE is 657.8655 m and 981.7151 m respectively
(maximum errors 1,397.1704 m and 1,847.8646 m). The poor outage behaviour is a
frozen regression datum, not a result to tune away during Phase 0.

For an accepted baseline:

1. Run each required profile three times through complete-bag deterministic drain.
2. Require exact component input counts matching the bag metadata.
3. Use IMU-triggered publication and require identical estimator sample counts.
4. Require zero late inputs and an empty reorder buffer after explicit drain.
5. Compare trajectories sample by sample without interpolation or alignment.
6. Require agreement within `1e-12` (the accepted baseline currently agrees exactly).
7. Treat a mismatch as an infrastructure failure, not an estimator regression.

## Profiles

The frozen core profiles are:

- `baseline`: legacy propagation and no wheel aid.
- `full`: continuous noise density, second-order transition/process-noise
  approximation, NHC, ZUPT/ZIHR, and robust GNSS.
- `wheel_auto`: forward wheel speed with automatically estimated scale.
- `wheel_nhc`: automatic wheel scale and simultaneous body-velocity/NHC update.
- `wheel_nhc_fixed`: calibrated fixed wheel scale and simultaneous NHC update.

The exact merged YAML and its SHA-256 are authoritative. A profile name alone
is not sufficient: the meaning of `wheel_nhc` changed during earlier
experiments.

## Evaluation intervals

| Name | Start | End | Alignment | Purpose |
|---|---:|---:|---|---|
| full trajectory | bag start | bag end | none | global failure and availability |
| continuous | 1229227026.6 | 1229227120.6 | translation only | local continuous accuracy after removing known datum/sensor translation |
| longest outage | 1229227824.6 | 1229227906.4 | none | absolute dead-reckoning drift |

SE(3) and Sim(3) alignment are forbidden for all absolute-position and outage
results. The evaluator uses a maximum reference interpolation gap of 0.2 s and
zero time offset unless the manifest explicitly records another value.

Required metrics are 3D/horizontal/vertical RMSE, yaw RMSE, maximum 3D error,
match ratio, and output sample count. Phase 6 extends these with RPE, outage
endpoint drift, reacquisition overshoot/settling, NIS, NEES, runtime, and memory.

## Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 kalman_filter_localization_ros2/scripts/run_urbannav_ablation.py \
  --input-bag .data/urbannav_tokyo/processed/odaiba_input_bag_wheel \
  --reference-csv .data/urbannav_tokyo/processed/odaiba_reference_wheel.csv \
  --output-dir .data/phase0/odaiba_reference \
  --base-profile kalman_filter_localization_ros2/param/profiles/urbannav_tokyo_tuned.yaml \
  --profiles-dir kalman_filter_localization_ros2/param/profiles \
  --profile baseline --profile full --profile wheel_auto \
  --profile wheel_nhc --profile wheel_nhc_fixed \
  --rate 10.0 --startup-delay 5.0 --require-complete-input
```

The output directory must be new. Retrying into an existing directory is
forbidden so that partial and completed runs cannot be mixed.

After three runs complete, apply the sample-level gate without interpolation
or trajectory alignment:

```bash
python3 kalman_filter_localization_ros2/scripts/verify_repeatability.py \
  .data/phase0/odaiba_batch_10x_run1/manifest.json \
  .data/phase0/odaiba_batch_10x_run2/manifest.json \
  .data/phase0/odaiba_batch_10x_run3/manifest.json \
  --profile baseline \
  --output-json .data/phase0/odaiba_batch_10x_repeatability.json
```

The verifier requires completed manifests, passed input-count gates, identical
input/config provenance and replay settings, identical sample counts and
timestamps, and per-sample `x/y/z/yaw` agreement within `1e-12` by default. It
reports the first mismatching sample and maximum absolute differences.

## Phase 0 exit gate

Phase 0 is complete only when:

- three deterministic-drain Odaiba repetitions pass the repeatability gate;
- Applanix and available Istanbul baselines have schema-v2 manifests;
- every result is tied to Git state, environment, input/config checksums, exact
  commands, output artifacts, and sample counts;
- the regression tables are committed to the research evaluation document;
- a genuinely unused final holdout sequence is registered before evaluation.
