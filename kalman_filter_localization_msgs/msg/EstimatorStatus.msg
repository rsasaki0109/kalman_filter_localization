# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# BSD-3-Clause license. See the repository LICENSE file.

uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK=1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_FAULT=3

uint8 MODE_UNINITIALIZED=0
uint8 MODE_INITIALIZING=1
uint8 MODE_OPEN_SKY=2
uint8 MODE_URBAN=3
uint8 MODE_OUTAGE=4
uint8 MODE_REACQUISITION=5
uint8 MODE_STATIONARY=6
uint8 MODE_FAULT=7

std_msgs/Header header

uint8 health
uint8 mode
string health_text
string mode_text
string summary

bool initialized
bool stationary
bool gnss_available
bool wheel_available
bool replay_enabled
bool numerical_invariant_ok
bool sensor_fault_isolation_enabled
bool imu_isolated
bool gnss_isolated
bool wheel_isolated
bool odom_isolated

float64 filter_time
float64 latest_imu_time
float64 input_age_sec
float64 position_stddev_m
float64 velocity_stddev_mps
float64 yaw_stddev_rad
float64 wheel_scale_factor

uint64 received_imu
uint64 received_gnss
uint64 received_gnss_velocity
uint64 received_wheel
uint64 received_odom
uint64 published_pose
uint64 accepted_measurements
uint64 rejected_measurements
uint64 numerical_failures
uint64 rewind_count
uint64 late_input_count
uint64 buffered_input_count
uint64 isolated_measurements
uint64 imu_fault_events
uint64 gnss_fault_events
uint64 wheel_fault_events
uint64 odom_fault_events
