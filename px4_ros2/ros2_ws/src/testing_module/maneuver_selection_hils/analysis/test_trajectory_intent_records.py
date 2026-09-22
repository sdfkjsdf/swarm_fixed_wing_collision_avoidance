import unittest
from types import SimpleNamespace

from trajectory_intent_records import iter_candidate_intents


def batch(count=7):
    candidates = [SimpleNamespace(
        candidate_id=i, candidate_input=[20., 100., 0., float(i)],
        candidate_input_revision=10+i, compressed_mean=list(range(18))) for i in range(7)]
    return SimpleNamespace(source_timestamp_us=123, selection_epoch=4,
        candidate_set_size=count, candidate_set_kind=0,
        initial_covariance=list(range(49)), initial_state=list(range(7)),
        initial_roll_setpoint_rad=0., source_execution_input=[20., float('nan'), 0., 4.],
        source_execution_input_available=True, candidates=candidates)


class CandidateRecordsTest(unittest.TestCase):
    def test_historical_candidate_is_unchanged(self):
        old = SimpleNamespace(candidate_id=3, initial_covariance=list(range(49)))
        self.assertIs(list(iter_candidate_intents(old))[0], old)

    def test_expands_shared_covariance_without_changing_candidates(self):
        message = batch()
        records = list(iter_candidate_intents(message))
        self.assertEqual(len(records), 7)
        for i, record in enumerate(records):
            self.assertEqual(record.candidate_id, i)
            self.assertEqual(record.source_timestamp_us, 123)
            self.assertEqual(record.selection_epoch, 4)
            self.assertIs(record.initial_covariance, message.initial_covariance)
            self.assertIs(record.initial_state, message.initial_state)
            self.assertIs(record.source_execution_input, message.source_execution_input)
            self.assertEqual(record.initial_roll_setpoint_rad, message.initial_roll_setpoint_rad)
            self.assertTrue(record.source_execution_input_available)
            self.assertFalse(hasattr(record, 'safe_rejoin_requested'))
            for field, value in vars(message.candidates[i]).items():
                self.assertIs(getattr(record, field), value)

    def test_ignores_unused_fixed_capacity_slots(self):
        message = batch(3)
        message.candidates[6].candidate_id = 255
        self.assertEqual(len(list(iter_candidate_intents(message))), 3)

    def test_rejects_invalid_count_and_duplicate_ids(self):
        for count in (0, 8):
            with self.assertRaises(ValueError):
                list(iter_candidate_intents(batch(count)))
        message = batch()
        message.candidates[6].candidate_id = 0
        with self.assertRaises(ValueError):
            list(iter_candidate_intents(message))


if __name__ == '__main__':
    unittest.main()
