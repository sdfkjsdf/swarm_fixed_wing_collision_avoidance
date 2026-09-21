"""Offline-only regression: a missing commit must not disappear in intersection."""
import unittest
from types import SimpleNamespace
from analyze_point_convergence_bag import decision_consensus_summary


def decision(epoch):
    return SimpleNamespace(
        coordination_qualified=True, local_selection_epoch=epoch,
        selected_v4_cutover=False, selected_candidate_valid_mask=31,
        selected_candidate_ids=[0] * 5,
        selected_candidate_input_revisions=[1] * 5,
        selected_candidate_source_timestamps_us=[1000] * 5)


class CommitCoverageTest(unittest.TestCase):
    def test_missing_node_is_not_hidden_by_perfect_common_tuple_ratio(self):
        records = [[(0.01 * v, decision(1))] for v in range(5)]
        for v in (0, 2, 3, 4):
            records[v].append((1.0, decision(2)))
        result = decision_consensus_summary(records, [0.1, 1.1])
        self.assertEqual(result['same_tuple_common_epoch_ratio'], 1.0)
        self.assertEqual(result['all_vehicle_commit_coverage_ratio'], 0.5)
        self.assertEqual(result['union_qualified_epoch_count'], 2)
        self.assertEqual(result['partial_commits'][0]['missing_vehicle_ids'], [1])
        self.assertAlmostEqual(result['observed_first_commit_spread_ms']['max'], 40.0)

    def test_empty_records_are_not_reported_as_success(self):
        result = decision_consensus_summary([[] for _ in range(5)], [])
        self.assertIsNone(result['all_vehicle_commit_coverage_ratio'])
        self.assertIsNone(result['observed_first_commit_spread_ms']['max'])


if __name__ == '__main__':
    unittest.main()
