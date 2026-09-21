import unittest
from analyze_stopped_pipeline_timing import analyze


SAMPLE = '''[stop-stage-begin],1,0,2,0
[stop-stage],1,1000,0,50000,2000000,3000000,7,1,1
[stop-stage],1,51000,0,50000,52000000,53000000,7,1,1
[stop-stage-end],0,2
[stop-pipeline-begin],1,0,2,0,2,0
[stop-pipeline],1000,1000000,1800000,3100000,200000,300000,2,1,1
[stop-pipeline],51000,51000000,51800000,53100000,200000,300000,2,1,1
[stop-belief],1000,900,500000,1500000,0,600000,1100000,1400000,1
[stop-belief],51000,50900,50500000,51500000,0,50600000,51100000,51400000,1
[stop-pipeline-end],0,2,2
'''


class PipelineTimingTest(unittest.TestCase):
    def test_decomposition_units_and_missing_middleware(self):
        r = analyze(SAMPLE)
        self.assertEqual(r['pipeline_dropped'], 0)
        self.assertEqual(r['callback_interval']['max_ms'], 50)
        self.assertEqual(r['enqueue_to_dispatch']['max_ms'], .5)
        self.assertEqual(r['middleware_received_available'], 0)
        self.assertEqual(r['middleware_to_callback_wall']['count'], 0)
        gap = r['largest_completion_gaps']['trajectory_refresh'][0]
        self.assertAlmostEqual(gap['gap_ms'], 50)
        self.assertAlmostEqual(gap['worker_busy_overlap_ms'], 2.1)
        self.assertAlmostEqual(gap['other_time_ms'], 47.9)

    def test_truncated_dump_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze(SAMPLE.replace('[stop-pipeline-end],0,2,2', ''))

    def test_remote_reconstruction_is_not_owner_input_processing(self):
        r = analyze(SAMPLE + '[stop-remote-worker],0,14,0,2,7000000,700000\n', 50000, 60000)
        self.assertEqual(r['remote_processing_scope'], 'complete-set installation on state owner only')
        remote = r['remote_reconstruction_thread']
        self.assertEqual(remote['handler_mean_ms'], .5)
        self.assertEqual(remote['handler_max_ms'], .7)
        self.assertEqual(remote['processed_packets'], 14)  # entire run, not the time window
        self.assertFalse(remote['includes_queue_wait'])
        self.assertIn('entire run', remote['scope'])
        self.assertEqual(r['remote_processing_per_pass']['max_ms'], .2)

    def test_remote_reconstruction_summary_rejects_wrong_vehicle(self):
        with self.assertRaises(ValueError):
            analyze(SAMPLE + '[stop-remote-worker],1,14,0,2,7000000,700000\n')

    def test_invalid_clock_order_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze(SAMPLE.replace('600000,1100000', '1600000,1100000'))

    def test_source_window(self):
        r = analyze(SAMPLE, 50000, 60000)
        self.assertEqual(r['accepted_beliefs'], 1)
        self.assertEqual(r['largest_completion_gaps']['trajectory_refresh'], [])

    def test_cross_clock_age_is_not_transport_latency(self):
        r = analyze(SAMPLE)
        self.assertNotIn('source_to_callback_wall', r)
        self.assertEqual(r['apparent_source_to_callback_age']['max_ms'], .5)
        self.assertFalse(r['source_age_clock_contract']['pure_transport_latency'])
        self.assertFalse(r['source_age_clock_contract']['px4_to_pc_clock_mapping_error_accounted_for'])
        self.assertFalse(r['source_age_clock_contract']['pc_to_pi_clock_offset_accounted_for'])

    def test_wall_clock_offset_does_not_change_computation(self):
        original = analyze(SAMPLE)
        shifted = analyze(SAMPLE.replace(',1500000,0,', ',61500000,0,')
                          .replace(',51500000,0,', ',111500000,0,'))
        self.assertEqual(original['stage_timing'], shifted['stage_timing'])
        self.assertEqual(original['callback_interval'], shifted['callback_interval'])
        self.assertAlmostEqual(shifted['apparent_source_to_callback_age']['max_ms'], 60.5)

    def test_negative_apparent_age_is_not_clamped(self):
        r = analyze(SAMPLE.replace(',1500000,0,', ',500000,0,'))
        self.assertEqual(r['apparent_source_to_callback_age']['median_ms'], 0)


if __name__ == '__main__':
    unittest.main()
