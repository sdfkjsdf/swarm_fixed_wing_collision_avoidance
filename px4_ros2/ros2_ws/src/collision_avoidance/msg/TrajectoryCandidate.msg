# Candidate-specific data within one TrajectoryIntentBatch.
uint8 candidate_id
# [V_cmd, h_cmd, h_dot_cmd, a_lat_cmd]; h_cmd may be NaN when unused.
float32[4] candidate_input
uint64 candidate_input_revision
# [pos_t0, vel_t0, pos_t1.5, pos_t3.0, pos_t4.5, vel_t4.5], NED.
float32[18] compressed_mean
