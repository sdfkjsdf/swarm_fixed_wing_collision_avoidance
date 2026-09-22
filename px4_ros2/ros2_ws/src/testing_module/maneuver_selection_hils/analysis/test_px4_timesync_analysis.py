import unittest

from analyze_px4_timesync import analyze, analyze_delivery, gain, synthetic


class TimesyncAnalysisTest(unittest.TestCase):
    def test_gain_endpoints(self):
        self.assertEqual(gain(0), .05)
        self.assertEqual(gain(500), .003)

    def test_known_clock_cadence_regression(self):
        self.assertGreater(synthetic(False)['max'], 60)
        self.assertLess(synthetic(True)['max'], .01)
        self.assertLess(synthetic(True, -1_600_000_000_000_000)['max'], .5)

    def test_replay_with_inferred_initial_skew(self):
        rows = []
        estimate = -1_600_000_000_000_000.
        skew = -5.
        for i in range(20):
            observed = -1_600_000_000_000_000 - 500 * i
            if i:
                previous = estimate
                estimate = .003 * observed + .997 * (estimate + skew)
                skew = .003 * (estimate - previous) + .997 * skew
            rows.append(dict(wall_ns=(i + 1) * 1_000_000_000,
                             local_midpoint_us=(i + 1) * 1_000_000,
                             observed_us=observed, estimated_us=int(estimate), rtt_us=0))
        result = analyze(rows, 0, 30_000_000_000)
        self.assertLess(result['legacy_replay_error_us']['max'], 10)
        self.assertEqual(result['operating_sample_intervals_ms']['median'], 1000)

    def test_nonincreasing_clock_is_not_silently_replayed(self):
        rows = [dict(wall_ns=i, estimated_us=1, observed_us=1,
                     local_midpoint_us=1, rtt_us=0) for i in range(3)]
        with self.assertRaisesRegex(ValueError, 'Non-increasing'):
            analyze(rows, 0, 3)

    def test_rejected_samples_are_not_assumed_accepted(self):
        rows = [dict(wall_ns=i, estimated_us=1, observed_us=1,
                     local_midpoint_us=i, rtt_us=10_000) for i in range(3)]
        with self.assertRaisesRegex(ValueError, 'rejected-RTT'):
            analyze(rows, 0, 3)


class DeliveryDecompositionTest(unittest.TestCase):
    @staticmethod
    def row(source=100_000, sample=90_000, steady=1_000_000,
            wall=160_000_000):
        return [source, sample, steady, wall, 0, steady, steady, steady, 1]

    def test_shared_mapping_bias_cancels_only_from_interobserver_difference(self):
        row = self.row()
        first = analyze_delivery([row], {(100_000, 90_000): [150_000_000]}, 0, 1_000_000_000)
        shifted = list(row)
        shifted[0] -= 40_000
        shifted[1] -= 40_000
        second = analyze_delivery([shifted], {(60_000, 50_000): [150_000_000]}, 0, 1_000_000_000)
        self.assertEqual(first['apparent_pi_source_age_ms']['median'], 60)
        self.assertEqual(second['apparent_pi_source_age_ms']['median'], 100)
        for report in [first, second]:
            self.assertEqual(report['pi_callback_minus_pc_bag_ms']['median'], 10)
            self.assertEqual(report['sample_to_publication_ms']['median'], 10)
            self.assertEqual(report['identity_max_error_ns'], 0)
            self.assertFalse(report['clock_contract']['pure_transport_latency'])

    def test_ambiguous_missing_and_different_sample_matches_are_not_guessed(self):
        rows = [self.row(sample=80_000), self.row(), self.row(source=110_000)]
        receipts = {(100_000, 90_000): [150_000_000, 151_000_000],
                    (110_000, 90_000): [150_000_000]}
        report = analyze_delivery(rows, receipts, 0, 1_000_000_000)
        self.assertEqual(report['matched_messages'], 1)
        self.assertEqual(report['missing_pc_receipt'], 1)
        self.assertEqual(report['ambiguous_pc_receipt'], 1)

    def test_negative_difference_preserved_and_pi_clock_step_identified(self):
        rows = [self.row(), self.row(source=120_000, steady=21_000_000, wall=260_000_000)]
        report = analyze_delivery(rows, {(100_000, 90_000): [170_000_000],
                                         (120_000, 90_000): [190_000_000]}, 0, 1_000_000_000)
        self.assertEqual(report['pi_callback_minus_pc_bag_ms']['min'], -10)
        event = report['largest_interobserver_gap']
        self.assertEqual(event['pc_receipt_interval_ms'], 20)
        self.assertEqual(event['pi_callback_steady_interval_ms'], 20)
        self.assertEqual(event['pi_wall_minus_steady_increment_ms'], 80)

    def test_empty_window_does_not_invent_latency(self):
        report = analyze_delivery([self.row()], {}, 200_000_000, 300_000_000)
        self.assertEqual(report['matched_messages'], 0)
        self.assertEqual(report['apparent_pi_source_age_ms'], {'count': 0})
        self.assertIsNone(report['largest_interobserver_gap'])


if __name__ == '__main__':
    unittest.main()
