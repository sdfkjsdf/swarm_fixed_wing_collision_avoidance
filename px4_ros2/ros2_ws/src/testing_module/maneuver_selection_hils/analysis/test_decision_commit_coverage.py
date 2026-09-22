"""Offline-only regression: a missing commit must not disappear in intersection."""
import unittest
from types import SimpleNamespace
from analyze_point_convergence_bag import decision_consensus_summary, startup_readiness_summary


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

    def test_startup_proposal_does_not_mean_ready(self):
        rows = {}
        for v in range(5):
            rows[f'/common/px4_{v}/maneuver_selection_decision'] = [
                (1_100_000_000, SimpleNamespace(proposal_valid=False,
                    coordination_qualified=False, command_execution_requested=False)),
                (1_500_000_000, SimpleNamespace(proposal_valid=True,
                    coordination_qualified=False, command_execution_requested=False)),
            ]
        result = startup_readiness_summary(rows, 1_000_000_000)
        self.assertIsNone(result['last_node_first_qualified_observed_s'])
        for node in result['vehicles']:
            self.assertAlmostEqual(node['first_valid_proposal_observed_s'], 0.5)
            self.assertIsNone(node['first_qualified_observed_s'])
        for v in range(5):
            rows[f'/common/px4_{v}/maneuver_selection_decision'].append(
                (1_520_000_000 + v * 1_000_000, SimpleNamespace(proposal_valid=True,
                    coordination_qualified=True, command_execution_requested=False)))
        result = startup_readiness_summary(rows, 1_000_000_000)
        self.assertAlmostEqual(result['last_node_first_qualified_observed_s'], .524)
        self.assertTrue(all(n['unqualified_execution_request_count'] == 0
                            for n in result['vehicles']))

    def test_missing_readiness_and_unauthorized_request_are_not_hidden(self):
        self.assertIsNone(startup_readiness_summary({}, 1)['last_node_first_qualified_observed_s'])
        result = startup_readiness_summary({
            '/common/px4_0/maneuver_selection_decision': [
                (10, SimpleNamespace(proposal_valid=True, coordination_qualified=False,
                                     command_execution_requested=True)),
                (20, SimpleNamespace(proposal_valid=True, coordination_qualified=True,
                                     command_execution_requested=False)),
            ]}, 30)
        self.assertEqual(result['vehicles'][0]['unqualified_execution_request_count'], 1)
        self.assertLess(result['vehicles'][0]['first_qualified_observed_s'], 0)
        self.assertIsNone(result['last_node_first_qualified_observed_s'])


if __name__ == '__main__':
    unittest.main()
