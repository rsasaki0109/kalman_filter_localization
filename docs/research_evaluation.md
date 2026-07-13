# Research evaluation

This evaluation separates basic open-sky validation from UrbanNav Odaiba
urban-canyon and GNSS-outage testing. Generated bags and CSV files live under
the ignored `.data/` directory; each run also writes the exact merged YAML and
`manifest.json` used to create it.

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
