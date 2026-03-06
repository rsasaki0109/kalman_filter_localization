# cycle15

## Goal

- Make attitude-oriented experiments easier to read.
- Check whether the new gyro-bias state has a usable parameter region for Istanbul bags 4-6.

## Tooling

- `run_open_data_sweep.py` now also writes:
  - `ranking_by_yaw_rmse_deg.csv`
  - `ranking_by_attitude_angle_rmse_deg.csv`
- The single-bag HTML report now links those files and shows top tables for them.

Smoke output:

- `/tmp/kfl_yaw_ranking_smoke/ranking_by_yaw_rmse_deg.csv`
- `/tmp/kfl_yaw_ranking_smoke/ranking_by_attitude_angle_rmse_deg.csv`
- `/tmp/kfl_yaw_ranking_smoke/open_data_report_20260306_174402.html`

## Bias micro-grid

Bag6 probe:

- output: `/tmp/kfl_bag6_bias_grid/summary.csv`
- best by RMSE:
  - `var_imu_gyro_bias=1e-10`
  - `tau_gyro_bias_sec=3600`
  - `rmse_3d_m=0.050369`
  - `yaw_rmse_deg=0.051925`
  - `attitude_angle_rmse_deg=0.174996`
- best by yaw:
  - `var_imu_gyro_bias=1e-08`
  - `tau_gyro_bias_sec=3600`
  - `yaw_rmse_deg=0.050769`
  - but `rmse_3d_m=0.115510`
- best by attitude angle:
  - `var_imu_gyro_bias=1e-08`
  - `tau_gyro_bias_sec=36000`
  - `attitude_angle_rmse_deg=0.170658`
  - but `yaw_rmse_deg=0.051624`

Bag4-5 safety probe with near-disabled bias process:

- bag4: `/tmp/kfl_bias_safe_eval/all-sensors-bag4_compressed/summary.csv`
- bag5: `/tmp/kfl_bias_safe_eval/all-sensors-bag5_compressed/summary.csv`
- params:
  - `var_imu_gyro_bias=1e-12`
  - `tau_gyro_bias_sec=3600`

## Conclusion

- The gyro-bias state has a narrow region where position can stay strong.
- But across bags 4-6, it still does not produce a robust attitude win.
- The bias-state implementation is worth keeping in code.
- It is still not ready to promote into the Istanbul profiles.
