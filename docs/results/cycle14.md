# cycle14

## Goal

- Move from parameter tuning to a state-model change for attitude improvement by introducing a gyro-bias state into the EKF.

## Implemented

- EKF state extended from `[p, v, q]` to `[p, v, q, b_g]`.
- Prediction now integrates angular velocity as `gyro - gyro_bias`.
- Added gyro-bias process parameters:
  - `var_imu_gyro_bias`
  - `tau_gyro_bias_sec`
- Added core tests for:
  - subtracting gyro bias during prediction
  - making gyro bias observable from repeated orientation observations

## Safety decision

- Default `var_imu_gyro_bias` is set to `0.0`.
- Gyro-bias covariance starts at zero.
- This keeps the new state effectively disabled unless a profile or experiment explicitly turns it on.

## Validation

Build/test:

- `colcon build --packages-select kalman_filter_localization_core kalman_filter_localization`
- `colcon test --packages-select kalman_filter_localization_core`
- result: `43 tests, 0 errors, 0 failures, 5 skipped`

Targeted runtime probes:

- Default bias settings (`var_imu_gyro_bias=1e-6`) were rejected: position regressed badly on bags 4 and 6.
- Bag6 grid over bias parameters showed that very small bias noise can preserve position:
  - best RMSE in the probe: `/tmp/kfl_bag6_bias_grid/summary.csv`
  - best params there: `var_imu_gyro_bias=1e-10`, `tau_gyro_bias_sec=3600`
- But the same probe did not deliver a robust yaw/attitude improvement relative to the current stable path.

## Conclusion

- Gyro-bias state groundwork is now in place.
- It is not promoted into the Istanbul profiles yet.
- The code path is available for further experiments, but there is still no validated cross-bag attitude win from it.
