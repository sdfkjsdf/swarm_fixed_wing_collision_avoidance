"""Offline candidate views for historical single-intent and atomic-batch bags.

No live compatibility publisher/subscriber: new nodes use only the batch schema.
The historical TrajectoryIntent ROS schema remains for reading earlier bags.
"""
from types import SimpleNamespace


def iter_candidate_intents(message):
    if not hasattr(message, 'candidates'):
        yield message
        return
    count = int(message.candidate_set_size)
    if not 1 <= count <= len(message.candidates):
        raise ValueError('Invalid trajectory batch count')
    candidates = message.candidates[:count]
    ids = [int(c.candidate_id) for c in candidates]
    if len(set(ids)) != count or any(not 0 <= i < 7 for i in ids):
        raise ValueError('Invalid trajectory batch candidate IDs')
    fields = ('candidate_id', 'candidate_input', 'candidate_input_revision',
              'compressed_mean')
    for candidate in candidates:
        yield SimpleNamespace(
            source_timestamp_us=message.source_timestamp_us,
            selection_epoch=message.selection_epoch,
            candidate_set_size=count,
            candidate_set_kind=message.candidate_set_kind,
            initial_covariance=message.initial_covariance,
            initial_state=message.initial_state,
            initial_roll_setpoint_rad=message.initial_roll_setpoint_rad,
            source_execution_input=message.source_execution_input,
            source_execution_input_available=message.source_execution_input_available,
            **{field: getattr(candidate, field) for field in fields})
