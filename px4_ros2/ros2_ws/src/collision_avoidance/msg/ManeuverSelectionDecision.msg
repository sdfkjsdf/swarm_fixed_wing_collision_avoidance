# Per-aircraft view of one distributed five-aircraft selection result.
uint64 selection_timestamp_us
int32 vehicle_id
uint8 aircraft_count
uint64 local_selection_epoch
uint64[5] selection_epochs_by_aircraft
uint16 selected_combination_index
uint16 evaluated_combination_count
uint8[5] selected_candidate_ids
uint8 ownship_candidate_id

# Locally evaluated tuple awaiting the distributed cross-check.  The selected
# fields above describe only the last tuple confirmed by every participant.
uint64 proposal_timestamp_us
uint64 proposal_epoch
uint8[5] proposed_candidate_ids
bool proposal_valid
bool proposal_consensus_confirmed

float32 pmr_m
float32 masd_m
float32 ad_m
float32 reciprocal_cost_sum
uint64 activation_timestamp_us
uint8 deactivation_reason
bool coordination_qualified
bool new_best_accepted
bool previous_best_retained
bool activation_requested
bool activation_just_started
bool activation_just_ended
