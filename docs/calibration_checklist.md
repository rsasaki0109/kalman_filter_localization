# Offline calibration checklist

Record every calibration input, tool version, command, output hash, dataset interval, and acceptance
threshold in the run manifest. Never tune these quantities on the blind holdout.

## IMU and vehicle frames

- Estimate the full IMU-to-base rotation from surveyed mounting geometry or a six-position static
  calibration. Verify ROS REP-103 FLU signs with independent positive roll, pitch, and yaw motions.
- Confirm accelerometer convention using a stationary sample: specific force must have norm close to
  gravity and map to world `+z` through the estimated body attitude.
- Confirm gyro signs with single-axis rotations and check timestamp monotonicity before estimating
  any bias.

## GNSS antenna

- Survey the base-to-antenna lever arm in the base FLU frame, including its sign and uncertainty.
- Validate the lever arm on turns in both directions; residuals must not change sign with yaw rate.
- Estimate GNSS-to-IMU time offset by cross-correlating Doppler/course dynamics with inertial yaw
  rate and longitudinal acceleration. Report the peak width, not only the selected offset.

## Wheel input

- Document the wheel message frame and whether speed is signed forward velocity or unsigned speed.
- Verify forward sign using a straight drive and reverse segment.
- Estimate scale only on straight, non-slip, sufficiently fast intervals; report median, spread, and
  sample count. Keep the scale fixed during the evaluation split.
- Check wheel-to-IMU time offset independently of the GNSS offset.

## Acceptance

- Re-run left/right turns, acceleration/braking, stationary, and reverse sanity segments.
- Store residual plots and machine-readable summary statistics.
- Reject a calibration when its uncertainty is unavailable, frame/sign convention is ambiguous, or
  estimates disagree materially across calibration intervals.

## Temperature availability

The registered UrbanNav Tokyo, Applanix open-sky, and Istanbul conversion bags contain IMU,
position, and wheel-related topics but no calibrated IMU temperature channel. Bias-versus-
temperature modelling is therefore not enabled for these datasets. A future dataset must first
show a repeatable held-out bias/temperature relationship before adding temperature states.
