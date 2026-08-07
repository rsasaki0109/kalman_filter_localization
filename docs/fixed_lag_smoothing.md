# Fixed-lag RTS smoother as an offline EKF oracle

Final update: 2026-08-08

## Overview

`kalman_filter_localization_core/include/kalman_filter_localization/core/fixed_lag_smoother.hpp`
implements a fixed-lag Rauch-Tung-Striebel (RTS) smoother over the ESKF. It is an
**offline oracle** for EKF error analysis (plan.md Phase 7): while a filter only
uses past measurements, the smoother re-conditions every retained state on the
measurements that arrived *after* it, so past states get corrected by future
information.

The smoothing track reuses the same `EKFEstimator` dynamics and records, for every
recorded node:

- the nominal state right after propagation (before any measurement update),
- the post-update state and covariance,
- the discrete transition / process-covariance pair `(Phi, Qd)` the filter
  actually used (`EKFEstimator::getLastDiscreteModel`).

A backward RTS pass then yields smoothed states that are consistent with the
filter's linearization.

## Math

For error-state ESKF with right-multiplicative quaternion error, the RTS backward
recursion over nodes `0..N-1` is:

```
P_{k+1|k} = Phi_k P_k Phi_k^T + Qd_k          (stored as predicted_covariance)
C_k       = P_k Phi_k^T (P_{k+1|k})^{-1}
x_k^s     = x_k (+) C_k ( x_{k+1}^s (-) x_{k+1|k} )
P_k^s     = P_k + C_k ( P_{k+1}^s - P_{k+1|k} ) C_k^T
```

where `(-)` is the error-state difference (position/velocity difference, `Log`
of the quaternion error, bias difference) and `(+)` is the error-state injection
(right quaternion multiplication), matching `EKFEstimator::applyErrorState`.

The last node is initialized to the filter estimate (`x_N^s = x_N`, `P_N^s = P_N`).

## Operation modes

- **Fixed-lag** (`lag_sec > 0`): the retained window is bounded in time. When the
  oldest node leaves the window it is smoothed against the current window,
  emitted through `popSmoothed()`, and dropped. Memory is `O(lag)`.
- **Batch** (`lag_sec <= 0`): all nodes are kept until `finalize()`, which runs
  one backward pass and emits the whole smoothed trajectory.

## Node subsampling

A plain fixed-lag RTS recomputes an `O(window)` backward pass per IMU step. At
100 Hz IMU and a 5 s window (500 nodes) this blocks the IMU callback, which in
turn starves the main replay (QoS depth 1 drops IMU and the filter wedges on
`dt > max_imu_dt`).

`setNodeSubsample(N)` records one node every `N`-th IMU sample and **composes**
the per-step discrete models over the interval:

```
Phi_composed = Phi_N * ... * Phi_1
Qd_composed  = Phi_N * ( ... ( Phi_2 * Qd_1 * Phi_2^T ) ... ) * Phi_N^T + ... + Qd_N
```

The composed pair is stored in the node, so one RTS step spans `N` IMU steps and
the window (and the per-step backward-pass cost) shrink by `N`. Node subsampling
does not change the filter's own propagation; `EKFEstimator` still integrates
every IMU sample.

## Measurement timing

Measurements whose sensor time equals the latest node boundary are applied
immediately and the node's post-update state/covariance is refreshed. All other
in-window measurements are deferred to the next node so that a node's
`predicted_state` / `predicted_covariance` stay measurement-free (as RTS
requires). Out-of-window, duplicate, and reverse-stamped events are counted and
rejected; delayed out-of-order rewind is the job of `EskfReplay` in the live
pipeline, not of this oracle.

## ROS 2 integration

`ekf_localization_node` (`ekf_localization_component.cpp`) exposes:

| Parameter | Default | Meaning |
|---|---|---|
| `enable_fixed_lag_smoothing` | `false` | enable the smoothing track |
| `fixed_lag_duration_sec` | `5.0` | smoothing lag window |
| `fixed_lag_node_subsample` | `1` | record one node every N IMU samples |

When enabled, the node runs a parallel `EKFEstimator` (`smoother_ekf_`) driven by
the same IMU stream and the same measurement functions as the main replay, and
publishes smoothed poses on:

```
/ekf_localization/smoothed_pose   (geometry_msgs/PoseStamped)
```

Smoothing requires `enable_measurement_replay: true` (the in-order,
sensor-time-aligned measurement feed). If smoothing is enabled without replay, a
warning is logged and the smoother stays inactive.

Initial pose / stationary initialization set the smoother's state to match the
main filter, and reset the smoother so the two tracks share the same starting
point.

## Real-data result

Applanix open-sky bag (120 s window), `research_smoothing.yaml` profile
(`propagation_model: fast`, replay enabled, subsample 10):

| Track | GNSS ground-truth mean 3D error |
|---|---:|
| EKF filter (`/ekf_localization/current_pose`) | 15.4 m |
| Fixed-lag smoothed (`/ekf_localization/smoothed_pose`) | 1.9 m |

The smoother improves the ground-truth error by roughly 8x by re-conditioning past
states on future GNSS updates.

The smoothing forward track runs its **own** stationary detector and vehicle model
(`smoother_stationary_detector_`, `smoother_vehicle_model_`), so the per-IMU
corrections (orientation / flat-ground / NHC / ZUPT / ZIHR / bias-observability
gating) are applied independently of the main filter. This keeps the two forward
runs from sharing stateful detector state while making their dynamics identical.
On the same Applanix segment, feeding these corrections reduced the
filter-vs-smoothed trajectory gap from ~29 m to ~15 m; the remaining gap comes
from the smoother fusing measurements at node boundaries (every `node_subsample`
IMU steps) instead of at each sensor timestamp as the replay engine does, and is
not an accuracy problem: the smoothed track stays ~8x closer to the reference.

## Limitations

- The smoother is an offline oracle; it is not used for control. Its forward
  track applies the same per-IMU corrections (via private detector / vehicle
  model instances) and the same measurements as the filter, so the only
  difference is the backward smoothing pass.
- Subsample composition is a first-order composition of the discrete models;
  large subsample factors reduce the effective node rate and slightly coarsen the
  smoothing timeline.
- Measurements are applied at node boundaries, so a measurement between nodes is
  fused at the next node (up to `node_subsample / imu_rate` later). This is the
  main source of the remaining filter-vs-smoothed gap; it does not hurt
  accuracy.
- Out-of-order delayed measurements are rejected by the smoother (the live
  pipeline keeps `EskfReplay` for those).
