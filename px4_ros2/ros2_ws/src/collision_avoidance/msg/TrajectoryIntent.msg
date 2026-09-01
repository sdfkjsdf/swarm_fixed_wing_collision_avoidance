# Compact A-to-B trajectory intent in the shared local NED frame.
uint64 source_timestamp_us
uint64 selection_epoch
uint8 candidate_id
uint8 candidate_set_size
uint8 candidate_set_kind

# Actual dynamic predictor input: [V_cmd, h_cmd, h_dot_cmd, a_lat_cmd].
# h_cmd may be NaN when altitude hold is intentionally unused.
float32[4] candidate_input
uint64 candidate_input_revision

# Input that remains in effect before the candidate can be applied, followed
# by the project-configured application delay represented in this rollout.
float32[4] current_input
float32 command_delay_s

# Predictor state order: [p_n, p_e, h, V_ground, course, h_dot, roll].
float32[7] initial_state

# Row-major 7x7 covariance of initial_state.
float32[49] initial_covariance

# [pos_t0, vel_t0, pos_t1.5, pos_t3.0, pos_t4.5, vel_t4.5],
# with three NED components per entry.
float32[18] compressed_mean
