# Time-aligned 4.5 s fixed-wing trajectory belief in the local NED frame.
uint64 timestamp
uint64 source_timestamp
uint64 source_timestamp_sample
float32 source_delay_s

float32 confidence_level
float32 chi_square_quantile
bool valid
uint8 point_count

# 46 samples at t = 0.0, 0.1, ..., 4.5 s.
float32[46] time_offset_s

# Point-major arrays. mean_position_ned has [north,east,down] per point.
# position_covariance_ned has a row-major 3x3 covariance per point.
float32[138] mean_position_ned
float32[414] position_covariance_ned

# Applied future input for each of the 45 propagation intervals.
# Order per interval: [V_cmd, h_cmd, h_dot_cmd, a_lat_cmd].
float32[180] prediction_inputs
