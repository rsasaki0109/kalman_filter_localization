# Allan variance conversion report

Input: `docs/allan_variance_example.json`

| Input coefficient | Unit | Value | EKF variance |
|---|---|---:|---:|
| gyro_white_noise_density | rad/s/sqrt(Hz) | 0.02 | `var_imu_w=0.0004` |
| accel_white_noise_density | m/s^2/sqrt(Hz) | 0.05 | `var_imu_acc=0.0025` |
| gyro_bias_random_walk | rad/s^2/sqrt(Hz) | 0.0001 | `var_imu_gyro_bias=1e-08` |
| accel_bias_random_walk | m/s^3/sqrt(Hz) | 0.0001 | `var_imu_acc_bias=1e-08` |

The EKF parameters are continuous-time power spectral densities: each value is the square of the
corresponding amplitude density. No sampling-rate factor is applied here.
