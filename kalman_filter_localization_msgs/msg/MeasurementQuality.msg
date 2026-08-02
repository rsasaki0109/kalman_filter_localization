# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# BSD-3-Clause license. See the repository LICENSE file.

std_msgs/Header header

uint64 sequence
uint8 source_id
string source
bool accepted
uint8 reject_reason
string reject_reason_text

float64 innovation_magnitude
float64 raw_nis
float64 used_nis
float64 variance_scale
