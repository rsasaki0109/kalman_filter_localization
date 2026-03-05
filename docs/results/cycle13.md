# cycle13

## Goal

- Verify whether attitude can be improved after the GNSS-velocity update without giving back the GT position gains on Istanbul bags 4-6.

## Kept

- Added `propagate_gnss_velocity_cross_state` as an explicit EKF/ROS parameter and kept the stable default at `true`.
- Reflected that default in the bag4-6 tuned profile and validator.

## Rejected

- `GNSS course yaw` as an additional explicit yaw observation:
  - it could improve `rmse_3d_m` on some runs, but did not produce a stable yaw/attitude gain against the reference.
- Partial orientation-gain scaling inside the GNSS-velocity update:
  - some sweep runs looked promising, but single-run reruns did not reproduce the same gains.
  - this knob was removed instead of being left as an unstable public parameter.

## Validation

Current stable path validation output:

- `/tmp/kfl_profile_validation_current_after_gain/profile_validation_summary.csv`
- `/tmp/kfl_profile_validation_current_after_gain/kf_profile_validation_report.html`

Bag metrics versus cycle12 baseline:

|bag|rmse_3d_m|yaw_rmse_deg|attitude_angle_rmse_deg|note|
|---|---:|---:|---:|---|
|bag4|`0.068701 -> 0.044765`|`0.067578 -> 0.061559`|`0.188423 -> 0.191510`|position improved strongly, yaw slightly improved, total attitude angle flat-to-slightly worse|
|bag5|`0.032177 -> 0.042034`|`0.023164 -> 0.021947`|`0.153904 -> 0.153335`|small position regression, negligible attitude improvement|
|bag6|`0.054656 -> 0.054672`|`0.050154 -> 0.050147`|`0.170201 -> 0.170490`|effectively unchanged|

## Conclusion

- The current stable algorithm still improves trajectory/position more than attitude.
- No tested `course yaw` or partial-coupling variant produced a robust attitude win worth promoting to the profile.
- The next real attitude-improvement step is not more tuning. It is a state/observation-model change such as:
  - gyro bias state augmentation
  - a more principled yaw observation model than raw GNSS course
