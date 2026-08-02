# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# BSD-3-Clause license. See the repository LICENSE file.

std_msgs/Header header

uint8 stationary_state
string stationary_state_text
uint8 slip_state
string slip_state_text
string vehicle_model
bool nhc_vertical_constrained

float64 nhc_variance_scale
float64 slip_severity
float64 wheel_innovation_mps
bool gyro_bias_learning_enabled
bool accel_bias_learning_enabled
