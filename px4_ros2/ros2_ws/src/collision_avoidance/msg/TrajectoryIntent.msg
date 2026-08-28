# Compact A-to-B trajectory intent in the shared local NED frame.
uint64 source_timestamp_us
uint8 candidate_id

# Predictor state order: [p_n, p_e, h, V_ground, course, h_dot, roll].
float32[7] initial_state

# Row-major 7x7 covariance of initial_state.
float32[49] initial_covariance

# [pos_t0, vel_t0, pos_t1.5, pos_t3.0, pos_t4.5, vel_t4.5],
# with three NED components per entry.
float32[18] compressed_mean
