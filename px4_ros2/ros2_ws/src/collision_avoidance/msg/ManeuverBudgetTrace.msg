# Observation only; never subscribed by the controller or peer coordination.
# event: 1 proposal ready; 2 commit; 3 pair monitor; 4 ROS publish bracket.
# wall_ns uses system UTC. steady_ns is comparable ONLY within the same host.
uint8 event
uint64 dropped_trace_count
uint64 wall_ns
uint64 steady_ns
uint64 publish_end_wall_ns
uint64 state_timestamp_us
uint64 state_sample_timestamp_us
uint64 evaluation_timestamp_us
uint64 epoch
int32 vehicle_id
int32 peer_id
uint8 candidate_id
uint8 peer_candidate_id
uint64 input_revision
uint64 peer_input_revision
uint64 source_timestamp_us
uint64 peer_source_timestamp_us
bool active
float64 pmr_m
float64 masd_m
float64 u95_m
float64 ad_m
float64 pmr_horizon_s
float64 lateral_acceleration_mps2
float64 ground_speed_command_mps
float64 equivalent_airspeed_command_mps
