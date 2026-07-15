# Research evaluation

This evaluation separates basic open-sky validation from UrbanNav Odaiba
urban-canyon and GNSS-outage testing. Generated bags and CSV files live under
the ignored `.data/` directory; each run also writes the exact merged YAML and
`manifest.json` used to create it.

## Phase 0 frozen regression baseline

The table below is the schema-v2, deterministic-drain baseline recorded before
the estimator mathematics redesign. These are regression observations, not
holdout claims or final acceptance thresholds. Full provenance and input-count
gates are in `docs/baseline_protocol.md` and the referenced manifests.

| Dataset/interval | Profile | 3D RMSE m | Horizontal m | Yaw RMSE deg | Evidence |
|---|---|---:|---:|---:|---|
| Applanix full trajectory | baseline | 14.6091 | 0.3056 | 1.3207 | `.data/phase0/applanix_batch_10x/manifest.json` |
| Applanix full trajectory | full | 19.0266 | 10.6587 | 8.2281 | `.data/phase0/applanix_batch_10x/manifest.json` |
| Odaiba continuous, translation aligned | baseline | 0.2470 | 0.2195 | 1.7641 | `.data/phase0/odaiba_segments/baseline_continuous.json` |
| Odaiba continuous, translation aligned | full | 0.2233 | 0.1916 | 1.6808 | `.data/phase0/odaiba_segments/full_continuous.json` |
| Odaiba 81.8 s outage, unaligned | baseline | 657.8655 | 634.1850 | 10.0219 | `.data/phase0/odaiba_segments/baseline_outage.json` |
| Odaiba 81.8 s outage, unaligned | full | 981.7151 | 981.6544 | 140.7849 | `.data/phase0/odaiba_segments/full_outage.json` |
| Istanbul bag4 | baseline / full | 0.0983 / 0.1562 | 0.0959 / 0.0902 | 2.7106 / 0.6711 | `.data/phase0/istanbul/bag4/results/manifest.json` |
| Istanbul bag5 | baseline / full | 0.0967 / 0.1346 | 0.0944 / 0.0689 | 4.6611 / 2.8273 | `.data/phase0/istanbul/bag5/results/manifest.json` |
| Istanbul bag6 | baseline / full | 0.1168 / 0.1651 | 0.1108 / 0.1083 | 2.6286 / 0.7042 | `.data/phase0/istanbul/bag6/results/manifest.json` |

Odaiba baseline and full each passed three-run sample-level repeatability with
62,040 poses and zero maximum difference in timestamp, position, or yaw.

## Phase 2 propagation verification

The `exact` backend uses midpoint nominal integration and Van Loan `Phi/Qd`.
Deterministic unit tests verify constant-model one-step/split-step equivalence,
PSD process covariance, and matching one-second covariance at 50, 100, 200,
and 400 Hz. A fixed-seed 4,000-trial, 15-dimensional Monte Carlo test places
mean NEES inside its three-sigma interval and closer to the expected value than
the frozen legacy discretization. A turning-acceleration case reduces midpoint
mean-state error below 15% of legacy error.

The ROS smoke manifest is `.data/phase2/istanbul_bag4_exact/manifest.json`.
It consumed 30 initial poses, 1,793 IMU samples, and 87 GNSS poses, emitted
1,793 poses with zero late/buffered inputs and no numerical rejection, and
reported 0.1559 m 3D RMSE and 0.6722 deg yaw RMSE.

## Phase 4 initialization and calibration

`ImuStationaryInitializer` uses a bounded IMU window to estimate gyro bias and the roll/pitch
rotation that aligns mean specific force with world `+z`. It reports sample count, gyro and
accelerometer standard deviations, gravity-norm error, a continuous stationary confidence, and a
machine-readable collecting/initialized/moving/invalid/reverse-time status. Yaw initialization has
an explicit priority: dual antenna, then Doppler/course, then external pose. Position, velocity,
attitude, gyro-bias, and accelerometer-bias prior covariance blocks are independently configurable.

On Istanbul bag4, the 0.5 s profile initialized from 50 samples with confidence 0.028 and gyro bias
`[-0.006651, -0.008654, 0.003454] rad/s`. The sensor-prior and wide-prior profiles both consumed all
1,793 IMU and 87 GNSS messages. Their 3D RMSE was 0.3066 m and 0.3033 m, respectively, while the
wide prior degraded yaw RMSE from 1.6944 deg to 6.5724 deg. A 1.0 s window failed on the same short
bag because it crossed into motion; the log reports gyro std 0.0184 rad/s, acceleration std 0.4263
m/s2, gravity error 0.2883 m/s2, and zero confidence. This establishes the short-bag window-length
dependency and provides a concrete failure diagnosis. Evidence is in
`.data/phase4/istanbul_bag4_initialization/manifest.json` and
`.data/phase4/istanbul_bag4_initialization_1s/initialization_1s/localization.log`.

`convert_allan_variance.py` validates unit-tagged amplitude densities and squares them into the
continuous-time EKF PSD parameters. A checked example and report live in
`docs/allan_variance_example.*`. `docs/calibration_checklist.md` records frame, lever-arm, wheel
sign/scale, and time-offset checks. The registered evaluation bags do not expose calibrated IMU
temperature, so no temperature model was added; that absence and the admission criterion for a
future model are documented in the checklist.

## Phase 5 measurement models and observability

Position, lever-arm position, world velocity, and yaw updates now compute their innovation and NIS
from the actual observation Jacobian and `S = H P H^T + R`. The position and velocity interfaces
accept full 3x3 covariance matrices. `NavSatFix.position_covariance` and Doppler twist covariance
retain valid off-diagonal terms; Doppler-derived course variance uses the complete horizontal 2x2
block. A shared quality gate assigns stable reject reasons for receiver quality, innovation, NIS,
reacquisition consistency, and numerical failures.

Doppler velocity/course is preferred whenever it was observed within the configured fallback
timeout. Position-difference velocity otherwise uses either propagated endpoint covariance or a
calibrated prior, followed by an explicit correlation inflation factor. The Istanbul evaluation
uses the calibrated-prior mode with a factor of two; its pre-inflation variance is half the frozen
effective variance, so it preserves the baseline measurement weight without pretending the two
position uses are independent. Position-difference course remains the yaw fallback.

The `/ekf_localization/debug/measurement_quality` array is machine-readable with fields
`[stamp, source, accepted, reject_reason, innovation_magnitude, raw_nis, used_nis]`. Sources are
1=position, 2=velocity, and 3=course. Reject reasons use the numeric order in
`MeasurementRejectReason`. `/debug/observability` reports stationary state, slip/turn state,
variance scale, severity, and wheel innovation.

`StationaryDetector` is shared by ZUPT and ZIHR. `SlipTurnDetector` has hysteretic trusted,
turning, and slip states and scales NHC variance. A deterministic injected-slip test confirms that
the slip update has lower NHC gain, the normal interval retains unit scale, and the detector
recovers after the configured sample count. Bias gain rows can be suppressed independently using
stationarity and angular/linear excitation decisions. GNSS reacquisition requires consecutive
position/velocity-consistent samples and then decreases variance over multiple updates; legacy
hard reset parameters are explicitly ignored.

Forward-wheel full cross-state and decoupled gains are selectable with
`wheel_speed_propagate_cross_state`; the corresponding profiles are `wheel_full_cross` and
`wheel_decoupled`. Unit ablation confirms that the full gain propagates velocity information into
attitude while the decoupled gain does not, and that the bias observability mask prevents bias
learning without excitation. Decoupled remains the conservative default; the attempted full
Odaiba deterministic-drain run exceeded the runner's 300 s service timeout and is excluded from
the quantitative evidence rather than treated as a result.

The fixed `observability` profile passed the Istanbul bag4-6 gate:

| Bag | Baseline 3D m | Phase 5 3D m | Change | Baseline yaw deg | Phase 5 yaw deg |
|---|---:|---:|---:|---:|---:|
| 4 | 0.09827 | 0.09844 | +0.17% | 2.7106 | 1.1109 |
| 5 | 0.09670 | 0.09684 | +0.15% | 4.6611 | 2.2718 |
| 6 | 0.11680 | 0.11683 | +0.02% | 2.6286 | 0.8469 |

Median yaw RMSE is 1.1109 deg versus the frozen 2.7106 deg, a 59.0% improvement, while every
3D position RMSE change is below 0.2%. Evidence is in
`.data/phase5/istanbul_bag{4,5,6}_calibrated_fallback/manifest.json`.

## Phase 6 evaluation, CI, and final verification

`evaluate_localization.py` now reports mean, median, RMSE, and maximum APE for 3D translation,
horizontal translation, vertical translation, and yaw. It also supports time- and
distance-indexed RPE and named GNSS-outage segments. An outage record contains endpoint drift,
post-reacquisition overshoot, settling time, and the threshold/window policy used to compute it.
Every result includes missing ratio plus the alignment, interpolation tolerance, time offset, and
evaluation interval. The ablation manifest stores the same policy and named segments, labels the
run as `tuning` or `holdout`, records the deterministic-drain timeout, and rejects a holdout
invocation containing more than one profile.

The fixed-seed C++ consistency test draws 2,000 independent state errors and observations from
known covariances. It checks 15-DoF state NEES, 3-DoF position NIS, 3-DoF velocity NIS, and 1-DoF
yaw NIS: each sample mean must be close to its DoF and each empirical 95% coverage must lie between
92% and 98%. `evaluate_consistency.py` provides the corresponding CSV reporting path; the checked
fixture summary is `.data/phase6/consistency_summary.json`. This gate uses the actual update
Jacobians and innovation covariances, so an accurate trajectory with systematically overconfident
covariance does not pass.

The normal CI job builds both packages and runs core unit tests, replay/delay tests, Monte Carlo,
Python evaluator tests, parameter-schema checks, and all linters. It then generates a real
rosbag2 fixture and replays 20 initial poses, 101 IMU samples, and 11 GNSS poses through the ROS
component. The smoke gate requires exact input counts, 101 output poses, no late or buffered
input, at least 95% reference matching, and bounded error. Full registered datasets are kept out
of pull-request CI; the scheduled/manual workflow downloads the registered benchmark and retains
manifests, comparisons, and localization logs as artifacts.

The final local suite completed with 220 tests, zero errors, zero failures, and 18 intentional
skips across both ROS packages.

The final Istanbul runs used the frozen `observability` profile after all estimator changes:

| Bag | 3D RMSE m | Horizontal m | Yaw RMSE deg | IMU/GNSS/output counts |
|---|---:|---:|---:|---:|
| 4 | 0.09844 | 0.09605 | 1.1129 | 1,793 / 87 / 1,793 |
| 5 | 0.09684 | 0.09450 | 2.2719 | 3,005 / 147 / 3,005 |
| 6 | 0.11683 | 0.11105 | 0.8496 | 2,031 / 99 / 2,031 |

All three input-count gates passed with match ratio 1.0 and zero late/buffered inputs. Their median
yaw RMSE is 1.1129 deg, 58.9% below the frozen median, and their 3D RMSE changes remain below 0.2%.
Evidence is `.data/phase6/istanbul_bag{4,5,6}_final/manifest.json`.

The bag4 runtime record reports real-time factor 1.193, mean/max IMU callback latency
4.50/10.06 ms, no retained history with replay disabled, and zero rewinds. Replay-enabled bag4
separately demonstrated 87 rewinds with bounded history in Phase 3. Three final-code bag4 runs are
sample-identical: both repeat comparisons contain 1,793 samples and maximum timestamp, position,
and yaw differences of exactly zero at tolerances 0 s and `1e-12`. The machine-readable result is
`.data/phase6/istanbul_bag4_repeatability.json`.

A replay-enabled CI-sized bag provides a direct memory record: 101 IMU and 11 GNSS inputs produced
101 outputs and 11 rewinds, retaining 61 IMU samples and six measurements in 5,656 bytes at
shutdown. Evidence is `.data/phase6/short_replay_performance_rate1/manifest.json`.

The final open-sky baseline exactly retained 14.6091 m 3D RMSE while improving yaw from 1.3207 to
1.2196 deg; all 85,344 IMU and 4,157 GNSS inputs produced 85,344 outputs. The final Odaiba
fixed-scale NHC run consumed 62,040 IMU, 62,040 wheel, and 3,790 GNSS samples without drops. It
uses a vertical-NHC variance scale of 20 while GNSS is available and restores the nominal variance
during outage. The scale sweep was restricted to the tuning split; 20 was the smallest sampled
value that passed both continuous gates and was selected instead of the best sampled metric.

The final fixed-scale result passes the urban-continuous 105% limits of 0.2594 m 3D and 1.8523 deg
yaw and improves the frozen 6.3358 m outage result:

| Profile | 3D RMSE m | Position gate | Yaw RMSE deg | Yaw gate |
|---|---:|---|---:|---|
| baseline | 0.24746 | pass | 1.9524 | fail |
| wheel auto-scale | 0.23615 | pass | 5.5053 | fail |
| wheel + NHC auto-scale, adaptive vertical variance | 0.27743 | fail | 1.3142 | pass |
| wheel + NHC fixed-scale, adaptive vertical variance | 0.25605 | pass | 1.3209 | pass |

Its 81.8 s outage 3D/horizontal/vertical RMSE is 3.1455/2.8457/1.3402 m. Reacquisition evaluation
reports 3.4184 m endpoint error and 11.5727 m overshoot; the configured 2 m for 2 s settling
criterion was not reached, and no acceptance threshold was defined for that diagnostic. A binary
outage-only vertical constraint was rejected because drift accumulated across short GNSS gaps, and
weakening course variance from 0.04 to 0.05 rad2 was rejected because it worsened outage RMSE.
The auto-scale companion passes its frozen outage gate at 6.0311 m versus 8.1117 m; it consumed the
same complete input counts with no late or buffered inputs. Its continuous 0.27743 m result is
reported as a failed diagnostic and is not the selected continuous profile.
Evidence is
`.data/phase6/applanix_final_baseline/manifest.json`,
`.data/phase6/odaiba_final_baseline/{manifest.json,continuous.json}`,
`.data/phase6/odaiba_final_wheel_auto/{manifest.json,continuous.json}`, and
`.data/phase6/odaiba_final_observability/{manifest.json,continuous.json,outage.json,outage_reacquisition.json}`.
Auto-scale outage evidence is
`.data/phase6/odaiba_final_auto_observability_drain1200/{manifest.json,continuous.json,outage.json}`.

## Open-sky validation

The local `driving_20_kmh` bag contains a 100 Hz Applanix GSOF49 trajectory and
vehicle speed. GSOF49 acceleration in this recording is gravity compensated:
its median stationary norm is 0.041 m/s2. The converter therefore restores
gravity in FLU body coordinates before publishing `sensor_msgs/Imu`. It also
applies the measured 1.69 s vehicle-speed timestamp offset and samples the
Applanix position at 5 Hz as the GNSS input.

```bash
source /opt/ros/jazzy/setup.bash
source /home/sasaki/applanix_ws/install/setup.bash
python3 kalman_filter_localization_ros2/scripts/prepare_applanix_open_sky.py \
  --input-db3 /home/sasaki/gnss_imu_wheel_localizer/data/driving_20_kmh/driving_20_kmh_2022_06_10-16_01_55_compressed_0.db3 \
  --output-bag .data/applanix_open_sky/input_bag_gravity \
  --output-reference-csv .data/applanix_open_sky/reference_gravity.csv \
  --gnss-rate-hz 5.0 --wheel-time-offset-sec 1.69 \
  --acceleration-mode gravity-compensated
```

At a common 10x playback rate, correcting the IMU interpretation reduced the
baseline 3D RMSE from 14.499 m to 0.256 m. Eagleye-style median scale estimation
reduced the forward-wheel result from 1.495 m to 0.588 m. The combined
forward/lateral/vertical observation is intentionally rejected for open-sky
use because its 7.874 m RMSE is worse than the one-axis model.

| Profile | 3D RMSE m | Horizontal m | Yaw RMSE deg |
|---|---:|---:|---:|
| baseline | 0.2560 | 0.1990 | 1.1363 |
| wheel fixed | 1.4947 | 1.3186 | 16.7601 |
| wheel auto-scale | 0.5878 | 0.5559 | 16.1894 |
| wheel + simultaneous NHC, auto-scale | 1.0945 | 0.7823 | 0.9446 |

The baseline is expected to win here because the same high-grade Applanix
solution supplies the 5 Hz position input and the reference. This run validates
the coordinate conversion, IMU axes/gravity convention, wheel timestamp, and
observation model; it is not an independent GNSS accuracy benchmark. The
auto-scale simultaneous-NHC profile is the portable fusion profile. The fixed
scale profile remains available as `wheel_nhc_fixed` for a calibrated vehicle.

## UrbanNav Odaiba

Run the selected profiles from the same wheel-enabled input bag:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 kalman_filter_localization_ros2/scripts/run_urbannav_ablation.py \
  --input-bag .data/urbannav_tokyo/processed/odaiba_input_bag_wheel \
  --reference-csv .data/urbannav_tokyo/processed/odaiba_reference_wheel.csv \
  --output-dir .data/urbannav_tokyo/processed/odaiba_final_eval \
  --base-profile kalman_filter_localization_ros2/param/profiles/urbannav_tokyo_tuned.yaml \
  --profiles-dir kalman_filter_localization_ros2/param/profiles \
  --profile baseline --profile wheel --profile wheel_auto \
  --profile wheel_nhc --profile wheel_nhc_fixed \
  --rate 10.0 --startup-delay 5.0
```

For the 94.0 s GNSS-continuous interval `1229227026.6` to `1229227120.6`, use
`compare_localization_results.py --start-stamp ... --end-stamp ...
--align-translation`. The median alignment removes the known datum/sensor
translation so this interval checks local consistency.

| Profile | Aligned 3D RMSE m | Horizontal m | Yaw RMSE deg |
|---|---:|---:|---:|
| baseline | 0.2484 | 0.2211 | 1.7764 |
| wheel fixed | 0.2281 | 0.2081 | 1.5567 |
| wheel auto-scale | 0.2210 | 0.2018 | 1.5484 |
| wheel + simultaneous NHC, auto-scale | 0.3154 | 0.2033 | 1.3478 |

For the longest 81.8 s GNSS outage, `1229227824.6` to `1229227906.4`, do not
align the trajectories. The combined body-velocity observation is selected for
outage suppression. The Eagleye-style scale estimate converges near 1.0 on
Odaiba and near 0.91 on the open-sky vehicle, allowing the same `wheel_nhc`
profile to run in both datasets.

| Profile | Outage 3D RMSE m | Horizontal m | Max 3D m |
|---|---:|---:|---:|
| baseline | 657.4691 | 633.7241 | 1395.4651 |
| wheel fixed | 427.3249 | 419.9573 | 930.7956 |
| wheel auto-scale | 420.7044 | 416.2136 | 924.8710 |
| wheel + simultaneous NHC, auto-scale | 8.1117 | 5.6921 | 9.4365 |
| wheel + simultaneous NHC, fixed scale | 6.3358 | 1.8712 | 7.8305 |

The generated trajectory CSV files are `<output-dir>/<profile>/estimate.csv`.
Machine-readable comparisons are `comparison.csv` and the segment CSV files
under `<output-dir>/segments/`.
## Phase 3 replay core verification

The ROS-free `EskfReplay` engine keeps a bounded history of IMU samples, complete estimator
snapshots (nominal state, covariance, and midpoint IMU memory), and measurement events. A delayed
event restores the anchor snapshot, interpolates IMU samples to the measurement time, applies the
update there, and repropagates every later event to the latest IMU time. Measurements older than
the retained anchor, future measurements, duplicate IDs, and reverse/duplicate IMU stamps have
separate status values and counters. Each measurement also records sensor, arrival, pre-update
filter, and apply times.

`test_eskf_replay` verifies exact state and covariance agreement against chronological fusion,
including reverse measurement order and a measurement between IMU samples. Its deterministic
delay sweep covers 0, 100, 250, and 500 ms with jitter, dropped measurements, and reordered
arrivals. For each case the replayed state and covariance agree with the no-delay run within
`1e-8`, and the latest output time remains monotonic. The bounded-history test also verifies
too-old, future, duplicate, and reverse-IMU policies. The Phase 3 core test run completed with
168 tests, zero failures, and 13 skips.

The ROS integration exposes `enable_measurement_replay`,
`measurement_history_duration_sec`, and `max_future_measurement_wait_sec`. GNSS position (including
the antenna lever arm), Doppler velocity, wheel speed, and odometry updates are stored as replayable
events. IMU orientation, flat-ground, NHC, ZUPT, and ZIHR decisions are retained as IMU-time
corrections so a rewind does not silently remove them. Near-future measurements are queued until
the IMU reaches their stamp; farther-future, too-old, duplicate, and reverse-IMU inputs have
separate statuses and counters. `compensate_gnss_delay` now emits a deprecation warning and is
superseded by replay when both are requested.

Istanbul bag4 was replayed at real-time rate with the Phase 3 runtime profile. The run consumed all
1,793 IMU and 87 GNSS messages, published 1,793 poses with strictly increasing stamps, performed 87
rewinds and only 86 IMU repropagations, and reported zero too-old, future, duplicate, rejected, or
numerical-failure events. Its 3D RMSE was 0.1295 m (horizontal 0.0702 m, maximum 0.6395 m). The
schema-v2 evidence is `.data/phase3/istanbul_bag4_replay_v4/manifest.json`.
