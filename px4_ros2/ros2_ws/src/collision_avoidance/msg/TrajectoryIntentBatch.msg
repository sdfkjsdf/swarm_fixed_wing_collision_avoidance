# One atomic trajectory-library refresh. No cross-message covariance cache.
uint64 source_timestamp_us
uint64 selection_epoch
uint8 candidate_set_size
uint8 candidate_set_kind
# Common source snapshot and last published input, not a hypothetical candidate.
# [p_n, p_e, h, V_ground, course, h_dot, roll].
float32[7] initial_state
float32 initial_roll_setpoint_rad
# [V_cmd, h_cmd, h_dot_cmd, a_lat_cmd]; h_cmd may be NaN when unused.
float32[4] source_execution_input
bool source_execution_input_available
# Shared row-major 7x7 INITIAL covariance, unchanged from the predictor.
# Future covariance is still propagated independently for each candidate.
float32[49] initial_covariance
# Fixed capacity avoids dynamic sequences; only candidate_set_size entries used.
TrajectoryCandidate[7] candidates
